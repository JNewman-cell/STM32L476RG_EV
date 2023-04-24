#include "SPI.h"

// Note: When the data frame size is 8 bit, "SPIx->DR = byte_data;" works incorrectly. 
// It mistakenly send two bytes out because SPIx->DR has 16 bits. To solve the program,
// we should use "*((volatile uint8_t*)&SPIx->DR) = byte_data";
void SPI1_GPIO_Init(void) {
	// TODO: initialize SPI1 GPIO pins
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	GPIOB->MODER &= ~GPIO_MODER_MODE3;
	GPIOB->MODER &= ~GPIO_MODER_MODE4;
	GPIOB->MODER &= ~GPIO_MODER_MODE5;
	
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL5;
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL3_2 | GPIO_AFRL_AFSEL3_0);
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0);
	
	GPIOB->MODER |= (GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_1);
	
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD4;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD5_0);
	
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT3;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT4;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT5;
	
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR3;
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4;
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5);
	
}

void SPI1_Init(void){
	// TODO: initialize SPI1 peripheral as master
	//(a) Enable the SPI clock.

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	//(b) Set the RCC SPI reset bit, then clear it to reset the SPI1 or SPI2 peripheral.

	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	//(c) Disable the SPI enable bit. The peripheral must be configured while it is disabled.

	SPI1->CR1 &= ~SPI_CR1_SPE;
	//(d) Configure the peripheral for full-duplex communication.

	SPI1->CR1 &= ~SPI_CR1_RXONLY;
	//(e) Configure the peripheral for 2-line unidirectional data mode.

	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	//(f) Disable output in bidirectional mode.

	SPI1->CR1 &= ~SPI_CR1_BIDIOE;
	//(g) Configure the frame format as MSB first.

	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
	//(h) Configure the frame format to 8-bit mode.

	SPI1->CR2 &= ~SPI_CR2_DS;
	SPI1->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 |SPI_CR2_DS_2);
	//(i) Use Motorola SPI mode.

	SPI1->CR2 &= ~SPI_CR2_FRF;
	//(j) Configure the clock to low polarity.

	SPI1->CR1 &= ~SPI_CR1_CPOL;
	//(k) Configure the clock to first clock transition.

	SPI1->CR1 &= ~SPI_CR1_CPHA;
	//(l) Set the baud rate prescaler to 2.

	SPI1->CR1 &= ~SPI_CR1_BR;

	//(m) Disable hardware CRC calculation.

	SPI1->CR1 &= ~SPI_CR1_CRCEN;
	//set to master mode

	SPI1->CR1 |= SPI_CR1_MSTR;
	//(o) Enable software SSM.

	SPI1->CR1 |= SPI_CR1_SSM;
	//(p) Enable NSS pulse generation.

	SPI1->CR2 |= SPI_CR2_NSSP;
	//(r) Set the FIFO threshold to 1/4 (required for 8-bit mode).

	SPI1->CR2 |= SPI_CR2_FRXTH;
	
	//configure internal slave select bit to 0
	
	SPI1->CR1 |= SPI_CR1_SSI;
	//(s) Enable the SPI peripheral.

	SPI1->CR1 |= SPI_CR1_SPE;
}
 
void SPI_Send_Data(SPI_TypeDef *SPIx, const uint8_t *write_data, uint32_t length) {
    // don't wait on the buffer if there's no transmission
    if (length == 0)
        return;

    // wait for transmission buffer to empty
    while (!(SPIx->SR & SPI_SR_TXE)) ;

    // write to the data register in packed mode if this is a multibyte transmission
    for (; length > 1; length -= sizeof(uint16_t)) {
        while (!(SPIx->SR & SPI_SR_TXE)) ;
        SPIx->DR = *((uint16_t *) write_data);
        write_data += sizeof(uint16_t);
    }

    // write the last (or only) byte if necessary
    if (length > 0)
        *((volatile uint8_t *) &SPIx->DR) = *write_data;

    // don't return until the SPI is no longer busy; otherwise weird stuff happens sometimes
    while (SPIx->SR & SPI_SR_BSY) ;
}
