#ifndef __STM32L476R_NUCLEO_SPI_H
#define __STM32L476R_NUCLEO_SPI_H

#include "stm32l476xx.h"

void SPI1_GPIO_Init(void);
void SPI1_Init(void);
void SPI_Send_Data(SPI_TypeDef *SPIx, const uint8_t *write_data, uint32_t length);

#endif 
