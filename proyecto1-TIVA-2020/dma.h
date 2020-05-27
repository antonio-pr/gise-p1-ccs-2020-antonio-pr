/*
 * dma.h
 *
 *  Created on: 21 may. 2020
 *      Author: usuario
 */

#ifndef DMA_H_
#define DMA_H_

#include<stdint.h>


#define MI_CANAL_DMA_A  17

void DMA_IniciaDMA(void);
void DMA_DMAError(void);
void DMASw_ISR(void);
void Espera_DMA(void);

#endif /* DMA_H_ */
