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
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "drivers/configADC.h"
#include "commands.h"
#include "semphr.h"

#include <remotelink.h>
#include <serialprotocol.h>


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


//Especificacion 2. Esta tarea no tendria por quï¿½ ir en main.c
static portTASK_FUNCTION(ADCTask,pvParameters)
{

    MuestrasADC muestras;
    MESSAGE_ADC_SAMPLE_PARAMETER parameter;

    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {

        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
        if(!resolution)
        {
            MESSAGE_ADC_SAMPLE_PARAMETER parameter;
            parameter.chan1=muestras.chan1;
            parameter.chan2=muestras.chan2;
            parameter.chan3=muestras.chan3;
            parameter.chan4=muestras.chan4;
            parameter.chan5=muestras.chan5;
            parameter.chan6=muestras.chan6;
            remotelink_sendMessage(MESSAGE_ADC_SAMPLE,(void *)&parameter,sizeof(parameter));
        }else
        {
            MESSAGE_ADC8_SAMPLE_PARAMETER parameter;
            parameter.chan1=(muestras.chan1>>4);
            parameter.chan2=(muestras.chan2>>4);
            parameter.chan3=(muestras.chan3>>4);
            parameter.chan4=(muestras.chan4>>4);
            parameter.chan5=(muestras.chan5>>4);
            parameter.chan6=(muestras.chan6>>4);
            remotelink_sendMessage(MESSAGE_ADC8_SAMPLE,(void *)&parameter,sizeof(parameter));
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
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);
                   ADCSequenceEnable(ADC0_BASE,0);
               }else if(parametro.index == 1)//GPIO DERECHO
               {
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_EXTERNAL,0);
                   GPIOADCTriggerDisable(GPIO_PORTF_BASE,GPIO_PIN_0);
                   GPIOADCTriggerEnable(GPIO_PORTF_BASE,GPIO_PIN_4);
                   GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
                   ADCSequenceEnable(ADC0_BASE,0);
               }else if(parametro.index == 2)//GPIO IZQUIERDO
               {
                   ADCSequenceDisable(ADC0_BASE,0);
                   ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_EXTERNAL,0);
                   GPIOADCTriggerDisable(GPIO_PORTF_BASE,GPIO_PIN_4);
                   GPIOADCTriggerEnable(GPIO_PORTF_BASE,GPIO_PIN_0);
                   GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);
                   ADCSequenceEnable(ADC0_BASE,0);
               }else if(parametro.index == 3)//TIMER
               {
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
                   ui32Period = SysCtlClockGet()/parametro.frecuencia;
                   TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
                   TimerEnable(TIMER2_BASE, TIMER_A);
                   ADCSequenceEnable(ADC0_BASE,0);
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

       case MESSAGE_RESOLUTION:
       {
           MESSAGE_RESOLUTION_PARAMETER parametro;
           if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
           {
               if(!parametro.resolution)//12 bits
               {
                   resolution=false;
               }else//8bits
               {
                   resolution=true;
               }
           }
           else
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
    uint32_t ui32Period;
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
	IntPrioritySet(INT_TIMER2A,configMAX_SYSCALL_INTERRUPT_PRIORITY);

	ui32Period = SysCtlClockGet();
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
	TimerEnable(TIMER2_BASE, TIMER_A);


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
    if((xTaskCreate(ADCTask, (portCHAR *)"ADC", ADC_TASK_STACK,NULL,ADC_TASK_PRIORITY, NULL) != pdTRUE))
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

