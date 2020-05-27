#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_comp.h"
#include "inc/hw_udma.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/comp.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


//Array destino del DMA (canal 6)
uint32_t buffer[512];

SemaphoreHandle_t semaforo_dma = NULL;




//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
// Array con tabla de control para el DMA
// Según la documentación, se le puede poner 512 de tamanho si no se usa el modo "ping-pong"
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif


//Canales DMA del ejemplo:





//Ejemplo configura los canales 6 y 30 del DMA
void DMA_IniciaDMA(void)
{

    //Habilita el DMA y le asigna la zona de memoria de configuración
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
    IntEnable(INT_UDMAERR);
    uDMAEnable();
    uDMAControlBaseSet(ui8ControlTable);

    //Configura el canal 17 para hacer transferencias disparadas por software (primera columna de la tabla del datasheet)
    uDMAChannelAssign(UDMA_CH17_ADC0_3);


    //*Configura un canales software del DMA (canal 30) */
    //Para transferir datos de g_ui16OrigBuf0 a g_ui16DstBuf0 */
    uDMAChannelAttributeDisable(MI_CANAL_DMA_A,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(MI_CANAL_DMA_A | UDMA_PRI_SELECT,
                          UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                          UDMA_ARB_1); //El tamaño de arbitración es el mismo que el del array. La transferencia se hace de una sóla ráfaga

    uDMAChannelTransferSet(MI_CANAL_DMA_A | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(ADC0_BASE + (ADC_O_SSFIFO3)),
                               ((void*)buffer), sizeof(buffer)/sizeof(uint32_t));

    uDMAChannelAttributeEnable(MI_CANAL_DMA_A, UDMA_ATTR_USEBURST);




    //Interrupciones del DMA (Software)
    //Ojo si se usa disparo hardware se deben habilitar LAS DEL PERIFÉRICO
    // (en el caso del ADC, las del sequencer correspondiente)!!!!
    IntEnable(INT_UDMA);
    IntPrioritySet(INT_UDMA,configMAX_SYSCALL_INTERRUPT_PRIORITY);

    semaforo_dma = xSemaphoreCreateBinary();
    if (semaforo_dma==NULL)
    {
        while(1);
    }
}

void Espera_DMA(void)
{
    xSemaphoreTake(semaforo_dma,portMAX_DELAY);
}

void ADC_SS3_RTI(void)
{
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    ADCIntClear(ADC0_BASE,3);//LIMPIAMOS EL FLAG DE INTERRUPCIONES




        /*uDMAChannelTransferSet(MI_CANAL_DMA_A | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC,
                                       (void *)(ADC0_BASE + (ADC_O_SSFIFO3)),
                                       ((void*)buffer), sizeof(buffer)/sizeof(uint32_t));

        uDMAChannelEnable(MI_CANAL_DMA_A);*/
        TimerDisable(TIMER2_BASE, TIMER_A);
        xSemaphoreGiveFromISR(semaforo_dma,&higherPriorityTaskWoken);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}


//Interrupcion de error de DMA, hay que ponerla por si acaso...
void DMA_DMAError(void)
{
    while(1)
    {
    }
}



