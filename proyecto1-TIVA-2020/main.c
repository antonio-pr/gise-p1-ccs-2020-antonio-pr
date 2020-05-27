//*****************************************************************************
//
// Codigo de partida Practica 1.
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "dma.h"

#include "drivers/rgb.h"
#include "drivers/configADC.h"
#include "commands.h"
#include "semphr.h"

#include <remotelink.h>
#include <serialprotocol.h>
#include<math.h>


//parametros de funcionamiento de la tareas
#define REMOTELINK_TASK_STACK (512)
#define REMOTELINK_TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define COMMAND_TASK_STACK (512)
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define ADC_TASK_STACK (256)
#define ADC_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define BUTTON_TASK_STACK (256)
#define BUTTON_TASK_PRIORITY (tskIDLE_PRIORITY+1)


//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;
bool resolution=false;
bool adc_mode=false;
bool simulation=false;
uint8_t contador=0;

extern uint32_t buffer[512];



SemaphoreHandle_t semaforo = NULL;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}


//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************
//Función
uint16_t simulate10bitADCsample(uint16_t canal)
{
    static uint16_t i[6] ={511,511,511,511,511,511};
    double a=3.0;

    canal = (canal % 6) +1;
    i[canal-1]++;
    return (1024.0*(0.46999999999*cos(canal*(a+0.14159265358979323846)*(i[canal-1]%512)/512.0)+0.49999999+0.0299999999*(rand()%200-100)/100.0));
}

//Especificacion 2. Esta tarea no tendria por quï¿½ ir en main.c
static portTASK_FUNCTION(ADCTask,pvParameters)
{

    MuestrasADC muestras;

    MESSAGE_ADC8_PARAMETER muestras8[16];
    MESSAGE_ADC_SAMPLE_PARAMETER muestras12[8];
    MESSAGE_ADC_SAMPLE_PARAMETER parameter;


    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {

        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
        if(!adc_mode)                   //Modo Software y GPIO
        {
            parameter.chan1=muestras.chan1;
            parameter.chan2=muestras.chan2;
            parameter.chan3=muestras.chan3;
            parameter.chan4=muestras.chan4;
            parameter.chan5=muestras.chan5;
            parameter.chan6=muestras.chan6;

            remotelink_sendMessage(MESSAGE_ADC_SAMPLE,(void *)&parameter,sizeof(parameter));
        }else//Modo Timer
        {
            if(!resolution)//12bits
            {
                if(simulation)
                {
                    muestras12[contador].chan1=simulate10bitADCsample(1);
                    muestras12[contador].chan2=simulate10bitADCsample(2);
                    muestras12[contador].chan3=simulate10bitADCsample(3);
                    muestras12[contador].chan4=simulate10bitADCsample(4);
                    muestras12[contador].chan5=simulate10bitADCsample(5);
                    muestras12[contador].chan6=simulate10bitADCsample(6);
                }else
                {

                    muestras12[contador].chan1=muestras.chan1;
                    muestras12[contador].chan2=muestras.chan2;
                    muestras12[contador].chan3=muestras.chan3;
                    muestras12[contador].chan4=muestras.chan4;
                    muestras12[contador].chan5=muestras.chan5;
                    muestras12[contador].chan6=muestras.chan6;
                }
                    contador++;
                    if(contador==8)
                    {
                        remotelink_sendMessage(MESSAGE_ADC12,(void *)&muestras12,sizeof(muestras12));
                        contador=0;
                    }
            }else
            {
                muestras8[contador].chan1=(muestras.chan1>>4);
                muestras8[contador].chan2=(muestras.chan2>>4);
                muestras8[contador].chan3=(muestras.chan3>>4);
                muestras8[contador].chan4=(muestras.chan4>>4);
                muestras8[contador].chan5=(muestras.chan5>>4);
                muestras8[contador].chan6=(muestras.chan6>>4);
                contador++;

                if(contador==16)
                {

                    remotelink_sendMessage(MESSAGE_ADC8,(void *)&muestras8,sizeof(muestras8));
                    contador=0;
                }
            }
        }
    }
}

static portTASK_FUNCTION(ADC_uDMATask,pvParameters)
{
    MESSAGE_DATA_ADQ_PARAMETER parametro[32];
    int i,j;
    while(1)
    {
        Espera_DMA(); //Bloqueante


        for(i=0;i<512/32;i++)
        {
            for(j=0;j<32;j++)
            {
                parametro[j].muestras=(uint16_t)buffer[j+(i*32)]>>4;
            }
            remotelink_sendMessage(MESSAGE_DATA_ADQ,(void *)&parametro,sizeof(parametro));
          //  vTaskDelay(1);
        }
    }
}


static portTASK_FUNCTION(ButtonTask,pvParameters)
{
    MESSAGE_BUTTON_PARAMETER parametro;
    uint32_t state;
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        xSemaphoreTake(semaforo,portMAX_DELAY);
        state = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4);

        if(!(state & GPIO_PIN_0))
        {
            parametro.right_button=true;
        }else
        {
            parametro.right_button=false;
        }

        if(!(state & GPIO_PIN_4))
        {
            parametro.left_button=true;
        }else
        {
            parametro.left_button=false;
        }
        remotelink_sendMessage(MESSAGE_BUTTON,(void *)&parametro,sizeof(parametro));
    }
}


//Funcion callback que procesa los mensajes recibidos desde el PC (ejecuta las acciones correspondientes a las ordenes recibidas)
static int32_t messageReceived(uint8_t message_type, void *parameters, int32_t parameterSize)
{
    int32_t status=0;   //Estado de la ejecucion (positivo, sin errores, negativo si error)

    //Comprueba el tipo de mensaje
    switch (message_type)
    {
        case MESSAGE_PING:
        {
            status=remotelink_sendMessage(MESSAGE_PING,NULL,0);
        }
        break;
        case MESSAGE_LED_GPIO:
        {
                MESSAGE_LED_GPIO_PARAMETER parametro;

                if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,parametro.value);
                }
                else
                {
                    status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                }
        }
        break;
        case MESSAGE_LED_PWM_BRIGHTNESS:
        {
            MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                RGBIntensitySet(parametro.rIntensity);
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_ADC_SAMPLE:
        {
            configADC_DisparaADC(); //Dispara la conversion (por software)
        }
        break;
        case MESSAGE_MODE:
        {
            MESSAGE_MODE_PARAMETER parametro;
            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                if(parametro.index == 0)
                {
                    RGBDisable();
                    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
                }else if(parametro.index == 1){
                    RGBEnable();
                }
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_COLOR:
        {
            MESSAGE_COLOR_PARAMETER parametro;
            uint32_t arrayRGB[3];
            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                arrayRGB[0]=(uint16_t) parametro.red << 8;
                arrayRGB[1]=(uint16_t) parametro.green << 8;
                arrayRGB[2]=(uint16_t) parametro.blue << 8;
                RGBColorSet(arrayRGB);
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
       break;

       case MESSAGE_BUTTON:
       {
           MESSAGE_BUTTON_PARAMETER parametro;
           uint32_t state = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4);
           if(!(state & GPIO_PIN_0))
           {
               parametro.right_button = true;

           }else{
               parametro.right_button = false;
           }

           if(!(state & GPIO_PIN_4))
           {
               parametro.left_button = true;
           }else{
               parametro.left_button = false;
           }

           status=remotelink_sendMessage(MESSAGE_BUTTON,&parametro,sizeof(parametro));
       }
       break;

       case MESSAGE_BUTTON_MODE:
       {
           MESSAGE_BUTTON_MODE_PARAMETER parametro;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(parametro.mode)
               {
                   GPIOADCTriggerDisable(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4);
                   GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_BOTH_EDGES);
                   GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
                   IntEnable(INT_GPIOF);
               }else
               {
                   IntDisable(INT_GPIOF);
               }
           }
           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_ADC_MODE:
       {
           MESSAGE_ADC_MODE_PARAMETER parametro;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(parametro.index == 0)//SOFTWARE
               {
                   adc_mode=false;
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);
                   ADCSequenceEnable(ADC0_BASE,0);
               }else if(parametro.index == 1)//GPIO DERECHO
               {
                   adc_mode=false;
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_EXTERNAL,0);
                   GPIOADCTriggerDisable(GPIO_PORTF_BASE,GPIO_PIN_0);
                   GPIOADCTriggerEnable(GPIO_PORTF_BASE,GPIO_PIN_4);
                   GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
                   ADCSequenceEnable(ADC0_BASE,0);
               }else if(parametro.index == 2)//GPIO IZQUIERDO
               {
                   adc_mode=false;
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_EXTERNAL,0);
                   GPIOADCTriggerDisable(GPIO_PORTF_BASE,GPIO_PIN_4);
                   GPIOADCTriggerEnable(GPIO_PORTF_BASE,GPIO_PIN_0);
                   GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);
                   ADCSequenceEnable(ADC0_BASE,0);
               }else if(parametro.index == 3)//TIMER
               {
                   adc_mode=true;
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_TIMER,0);
                   TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
               }
           }
           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_FACTOR:
       {
           MESSAGE_FACTOR_PARAMETER parametro;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               ADCSequenceDisable(ADC0_BASE,0);
               ADCHardwareOversampleConfigure(ADC0_BASE,parametro.factor);
               ADCSequenceEnable(ADC0_BASE,0);
           }
           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_TIMER_ADC:
       {
           MESSAGE_TIMER_ADC_PARAMETER parametro;
           uint32_t ui32Period;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(parametro.on)
               {
                   if(simulation)
                   {
                       ui32Period = SysCtlClockGet()/500;
                       TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
                       TimerEnable(TIMER2_BASE, TIMER_A);
                       ADCSequenceEnable(ADC0_BASE,0);
                   }else
                   {
                       ui32Period = SysCtlClockGet()/parametro.frecuencia;
                       TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
                       TimerEnable(TIMER2_BASE, TIMER_A);
                       ADCSequenceEnable(ADC0_BASE,0);
                   }

               }else
               {
                   TimerDisable(TIMER2_BASE, TIMER_A);
               }
           }
           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_RESOLUTION:
       {
           MESSAGE_RESOLUTION_PARAMETER parametro;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(!parametro.resolution)//12 bits
               {
                   resolution=false;
                   contador=0;
               }else//8bits
               {
                   resolution=true;
                   contador=0;
               }
           }
           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_SIMULATION:
       {
           MESSAGE_SIMULATION_PARAMETER parametro;
           adc_mode=true;
           resolution=false;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(parametro.simulation)
               {
                   simulation=true;
                   adc_mode=true;
                   resolution=false;
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_TIMER,0);
                   TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
               }else
               {
                   simulation=false;
                   TimerDisable(TIMER2_BASE, TIMER_A);
               }
           }

           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_DATA_ADQ_MODE:
       {
           MESSAGE_DATA_ADQ_MODE_PARAMETER parametro;

           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(parametro.mode==0)//ADQUISICIÓN DE DATOS
               {
                   ADCSequenceDisable(ADC0_BASE,3);
                   ADCSequenceDisable(ADC0_BASE,0);

                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);   //Disparo software (processor trigger)
                   ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH0);
                   ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH1);
                   ADCSequenceStepConfigure(ADC0_BASE,0,2,ADC_CTL_CH2);
                   ADCSequenceStepConfigure(ADC0_BASE,0,3,ADC_CTL_CH3);
                   ADCSequenceStepConfigure(ADC0_BASE,0,4,ADC_CTL_CH4);
                   ADCSequenceStepConfigure(ADC0_BASE,0,5,ADC_CTL_CH5);
                   ADCSequenceStepConfigure(ADC0_BASE,0,6,ADC_CTL_CH4);
                   ADCSequenceStepConfigure(ADC0_BASE,0,7,ADC_CTL_CH5|ADC_CTL_IE |ADC_CTL_END); //La ultima muestra provoca la interrupcion
                   ADCSequenceEnable(ADC0_BASE,0); //ACTIVO LA SECUENCIA

                   //Habilita las interrupciones
                   ADCIntClear(ADC0_BASE,0);
                   ADCIntDisable(ADC0_BASE,3);
                   ADCIntEnable(ADC0_BASE,0);
                   IntEnable(INT_ADC0SS0);
               }else //OSCILOSCOPIO
               {
                   //CONFIGURAR SECUENCIADOR 3
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceDisable(ADC0_BASE,3);

                   ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

                   ADCSequenceConfigure(ADC0_BASE,3,ADC_TRIGGER_TIMER,0);
                   ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH1|ADC_CTL_IE |ADC_CTL_END);
                   TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
                   ADCSequenceEnable(ADC0_BASE,3);




                   //Habilita las interrupciones
                   ADCIntDisable(ADC0_BASE,0);
                   ADCIntClear(ADC0_BASE,3);
                   ADCIntEnable(ADC0_BASE,3);
                   IntPrioritySet(INT_ADC0SS3,configMAX_SYSCALL_INTERRUPT_PRIORITY);
                   IntEnable(INT_ADC0SS3);
               }
           }
           else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }
       }
       break;

       case MESSAGE_DATA_ADQ_CAPTURE:
       {
           MESSAGE_DATA_ADQ_CAPTURE_PARAMETER parametro;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               IntEnable(INT_ADC0SS3);
               uDMAChannelTransferSet(MI_CANAL_DMA_A | UDMA_PRI_SELECT,
                                      UDMA_MODE_BASIC,
                                      (void *)(ADC0_BASE + (ADC_O_SSFIFO3)),
                                      ((void*)buffer), sizeof(buffer)/sizeof(uint32_t));
               uDMAChannelEnable(MI_CANAL_DMA_A);

               TimerLoadSet(TIMER2_BASE, TIMER_A, (SysCtlClockGet()/parametro.frecuencia) -1);
               TimerEnable(TIMER2_BASE, TIMER_A);
           }else
           {
               status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
           }

       }
       break;


       default:
           //mensaje desconocido/no implementado
           status=PROT_ERROR_UNIMPLEMENTED_COMMAND; //Devuelve error.
    }
    return status;   //Devuelve status
}


//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{
	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	MAP_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1
	RGBInit(1);
	MAP_SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	MAP_SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	MAP_SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

	//Volvemos a configurar los LEDs en modo GPIO POR Defecto
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	//Configuración del TIMER2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	//IntPrioritySet(INT_TIMER2A,configMAX_SYSCALL_INTERRUPT_PRIORITY);


	//Configuración de los PF4 Y PF0 como botones y activación de las interrupciones
	ButtonsInit();
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
	GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_BOTH_EDGES);
	IntPrioritySet(INT_GPIOF, configMAX_SYSCALL_INTERRUPT_PRIORITY);
	GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
	IntMasterEnable();

	semaforo = xSemaphoreCreateBinary();


	/********************************      Creacion de tareas *********************/

	//Tarea del interprete de comandos (commands.c)
    if (initCommandLine(COMMAND_TASK_STACK,COMMAND_TASK_PRIORITY) != pdTRUE)
    {
        while(1);
    }

    //Tarea de gestión de la interrupción de botones
    if((xTaskCreate(ButtonTask, (portCHAR *)"Button", BUTTON_TASK_STACK,NULL,BUTTON_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

	//Esta funcion crea internamente una tarea para las comunicaciones USB.
	//Ademas, inicializa el USB y configura el perfil USB-CDC
	if (remotelink_init(REMOTELINK_TASK_STACK,REMOTELINK_TASK_PRIORITY,messageReceived)!=pdTRUE)
	{
	    while(1); //Inicializo la aplicacion de comunicacion con el PC (Remote). Ver fichero remotelink.c
	}


	//Especificacion 2: Inicializa el ADC y crea una tarea...
	configADC_IniciaADC();
	DMA_IniciaDMA();
    if((xTaskCreate(ADCTask, (portCHAR *)"ADC", ADC_TASK_STACK,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(ADC_uDMATask, (portCHAR *)"ADC_uDMA", ADC_TASK_STACK,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }




	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}


void RutinaBotones(void)
{
    BaseType_t HigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(semaforo,&HigherPriorityTaskWoken);

    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);

    portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

