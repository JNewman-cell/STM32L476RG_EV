#include "Display.h"
#include "SysTimer.h"
#include "SPI.h"
// Global variables
static int fch; // Foreground color upper byte
static int fcl; // Foreground color lower byte
static int bch; // Background color upper byte
static int bcl; // Background color lower byte
static int X1, X2; // horizontal drawing boundaries
static int Y1, Y2; // vertical drawing boundaries
static struct current_font cfont;

//Uses PB3(SCK) PB5(MOSI) PA10(DC) PB10(CS) PA8(RST)
void Display_GPIO_Init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	GPIOB->MODER &= ~GPIO_MODER_MODE3;
	GPIOB->MODER &= ~GPIO_MODER_MODE5;
	
	//SCK and MOSI setup for pins PB3 and PB5
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL5;
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL3_2 | GPIO_AFRL_AFSEL3_0);
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0);
	
	//turn on the MOSI and SCK pins
	GPIOB->MODER |= (GPIO_MODER_MODE3_1 | GPIO_MODER_MODE5_1);
	
	//no pull up or down resistors
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10;
	
	//General purpose for DC CS and RST functions
	GPIOA->MODER &= ~GPIO_MODER_MODE8; //RST
	GPIOA->MODER &= ~GPIO_MODER_MODE10; //DC
	GPIOB->MODER &= ~GPIO_MODER_MODE10; //CS
	
	GPIOA->MODER |= GPIO_MODER_MODE8_0 | GPIO_MODER_MODE10_0;
	GPIOB->MODER |= GPIO_MODER_MODE10_0;
	
	//set output type of pins
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT8;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT3;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT5;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;

	//very high output speeds
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR10);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR10);
}

void LCD_Write_COM(uint8_t cmd){
	GPIOA->ODR &= ~GPIO_ODR_OD10;
	GPIOB->ODR &= ~GPIO_ODR_OD10;
	SPI_Send_Data(SPI1, &cmd, 1);
	GPIOB->ODR |= GPIO_ODR_OD10;
}

void LCD_Write_DATA16(uint8_t high, uint8_t low) {
	//DC
	GPIOA->ODR |= GPIO_ODR_OD10;
	//CS
	GPIOB->ODR &= ~GPIO_ODR_OD10;
  uint8_t pData[2] = {high, low};
  SPI_Send_Data(SPI1, pData, 2);
	GPIOB->ODR |= GPIO_ODR_OD10;
}

// Write 8-bit data to LCD controller
void LCD_Write_DATA(uint8_t data) {
	GPIOA->ODR |= GPIO_ODR_OD10;
	GPIOB->ODR &= ~GPIO_ODR_OD10;
  SPI_Send_Data(SPI1, &data, 1);
	GPIOB->ODR |= GPIO_ODR_OD10;
}

void Display_init(SPI_TypeDef *SPIx) {
  // (Display off)
  //hspi_cmd(SPIx, 0x28);
	//enable
	GPIOA->ODR |= GPIO_ODR_OD8;
	//spi init
	GPIOB->ODR |= GPIO_ODR_OD10;
	//reset
	GPIOA->ODR &= ~GPIO_ODR_OD8;
	delay(1);
	//CS off
	GPIOB->ODR &= ~GPIO_ODR_OD10;
	delay(1);
	GPIOA->ODR |= GPIO_ODR_OD8;
	
  // Reset
    LCD_Write_COM(0x01);
    delay(160); //Must wait > 5ms

    LCD_Write_COM(0xCB);
    LCD_Write_DATA(0x39);
    LCD_Write_DATA(0x2C);
    LCD_Write_DATA(0x00);
    LCD_Write_DATA(0x34);
    LCD_Write_DATA(0x02);

    LCD_Write_COM(0xCF);
    LCD_Write_DATA(0x00);
    LCD_Write_DATA(0XC1);
    LCD_Write_DATA(0X30);

    LCD_Write_COM(0xE8);
    LCD_Write_DATA(0x85);
    LCD_Write_DATA(0x00);
    LCD_Write_DATA(0x78);

    LCD_Write_COM(0xEA);
    LCD_Write_DATA(0x00);
    LCD_Write_DATA(0x00);

    LCD_Write_COM(0xED);
    LCD_Write_DATA(0x64);
    LCD_Write_DATA(0x03);
    LCD_Write_DATA(0X12);
    LCD_Write_DATA(0X81);

    LCD_Write_COM(0xF7);
    LCD_Write_DATA(0x20);

    LCD_Write_COM(0xC0);   //Power control
    LCD_Write_DATA(0x23);  //VRH[5:0]

    LCD_Write_COM(0xC1);   //Power control
    LCD_Write_DATA(0x10);  //SAP[2:0];BT[3:0]

    LCD_Write_COM(0xC5);   //VCM control
    LCD_Write_DATA(0x3e);  //Contrast
    LCD_Write_DATA(0x28);

    LCD_Write_COM(0xC7);   //VCM control2
    LCD_Write_DATA(0x86);  //--

    LCD_Write_COM(0x36);   // Memory Access Control
    LCD_Write_DATA(0x48);

    LCD_Write_COM(0x3A);
    LCD_Write_DATA(0x55);

    LCD_Write_COM(0xB1);
    LCD_Write_DATA(0x00);
    LCD_Write_DATA(0x18);

    LCD_Write_COM(0xB6);   // Display Function Control
    LCD_Write_DATA(0x08);
    LCD_Write_DATA(0x82);
    LCD_Write_DATA(0x27);


    LCD_Write_COM(0xF2); // 3GAMMA Function Disable
    LCD_Write_DATA(0x00);

    LCD_Write_COM(0x26); // Gamm Curve Selected
    LCD_Write_DATA(0x01);

    LCD_Write_COM(0xE0); // Positive Gamma Correction
    LCD_Write_DATA(0x0F);
    LCD_Write_DATA(0x31);
    LCD_Write_DATA(0x2B);
    LCD_Write_DATA(0x0C);
    LCD_Write_DATA(0x0E);
    LCD_Write_DATA(0x08);
    LCD_Write_DATA(0x4E);
    LCD_Write_DATA(0xF1);
    LCD_Write_DATA(0x37);
    LCD_Write_DATA(0x07);
    LCD_Write_DATA(0x10);
    LCD_Write_DATA(0x03);
    LCD_Write_DATA(0x0E);
    LCD_Write_DATA(0x09);
    LCD_Write_DATA(0x00);

    LCD_Write_COM(0xE1); // Negative Gamma Correction
    LCD_Write_DATA(0x00);
    LCD_Write_DATA(0x0E);
    LCD_Write_DATA(0x14);
    LCD_Write_DATA(0x03);
    LCD_Write_DATA(0x11);
    LCD_Write_DATA(0x07);
    LCD_Write_DATA(0x31);
    LCD_Write_DATA(0xC1);
    LCD_Write_DATA(0x48);
    LCD_Write_DATA(0x08);
    LCD_Write_DATA(0x0F);
    LCD_Write_DATA(0x0C);
    LCD_Write_DATA(0x31);
    LCD_Write_DATA(0x36);
    LCD_Write_DATA(0x0F);

    LCD_Write_COM(0x11);   //Exit Sleep
    delay(100); // wait > 10ms

    LCD_Write_COM(0x29);   //Display on
//    LCD_Write_COM(0x2c);

    // set rotation
    LCD_Write_COM(0x36);
    delay(1);
    LCD_Write_DATA(0x40|0x08);
		
		// Default color and fonts
	fch = 0xFF;
	fcl = 0xFF;
	bch = 0x00;
	bcl = 0x00;
	LCD_setFont(BigFont);
}

// Set boundary for drawing
void LCD_setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    X1 = x1;
    X2 = x2;
    Y1 = y1;
    Y2 = y2;

    LCD_Write_COM(0x2A);
    LCD_Write_DATA16(x1 >> 8U, x1);
    LCD_Write_DATA16(x2 >> 8U, x2);

    LCD_Write_COM(0x2B);
    LCD_Write_DATA16(y1 >> 8U, y1);
    LCD_Write_DATA16(y2 >> 8U, y2);

    LCD_Write_COM(0x2C);
}

// Set foreground RGB color for next drawing
void LCD_setColor(uint8_t r, uint8_t g, uint8_t b) {
    // 5-bit r, 6-bit g, 5-bit b
    fch = (r & 0x0F8) | g >> 5;
    fcl = (g & 0x1C) << 3 | b >> 3;
}

// Set background RGB color for next drawing
void LCD_setColorBg(uint8_t r, uint8_t g, uint8_t b) {
    // 5-bit r, 6-bit g, 5-bit b
    bch = (r & 0x0F8) | g >> 5;
    bcl = (g & 0x1C) << 3 | b >> 3;
}

// Clear display
void LCD_clrScr(void) {
    // Black screen
    LCD_setColor(0, 0, 0);
    LCD_fillRect(0, 0, DISP_X_SIZE, DISP_Y_SIZE);
}

// Remove drawing boundary
void LCD_clrXY(void) {
    X1 = X2 = Y1 = Y2 = 0;
    LCD_setXY(0, 0, 0, 0);
}

// Draw horizontal line
void LCD_drawHLine(uint16_t x, uint16_t y, int l) {
    if (l < 0) {
        l = -l;
        x -= l;
    }
	int i;
    LCD_setXY(x, y, x + l, y);
    for (i = 0; i < l + 1; i++) {
			LCD_Write_DATA(fch);
			LCD_Write_DATA(fcl);
		}

    LCD_clrXY();
}

// Quickly fill a rectangular area
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	int i;
    if (x1 > x2)
        swap(uint16_t, x1, x2);

    if (y1 > y2)
        swap(uint16_t, y1, y2);

    LCD_setXY(x1, y1, x2, y2);
    for (i = 0; i < (x2 - x1 + 1) * (y2 - y1 + 1); i++) {
			LCD_Write_DATA(fch);
			LCD_Write_DATA(fcl);
		}

    LCD_clrXY();
}

// Fill a triangle
void LCD_fillTriangle(uint16_t x, uint16_t y, int w, int h) {
//	xil_printf("draw triangle\n\r");
    short fill_dir;
    if (h < 0) {
        fill_dir = 1;
    } else {
        fill_dir = -1;
    }
    y -= h;

    float delta = w / (fill_dir * h * -2.0);
    float l = x, u = x + w;
    int i = h;
    while (i != 0) {
        LCD_drawHLine((int) l, y + i, (int) u - l);
        l += delta;
        u -= delta;
        i += fill_dir;
    }
}

// Select the font used by print() and printChar()
void LCD_setFont(uint8_t *font) {
    cfont.font = font;
    cfont.x_size = font[0];
    cfont.y_size = font[1];
    cfont.offset = font[2];
    cfont.numchars = font[3];
}

// Print a character
void LCD_printChar(uint8_t c, uint16_t x, uint16_t y) {
    uint8_t ch;
    int i, j, pixelIndex;

    LCD_setXY(x, y, x + cfont.x_size - 1, y + cfont.y_size - 1);

    pixelIndex = (c - cfont.offset) * (cfont.x_size >> 3) * cfont.y_size + 4;
    for (j = 0; j < (cfont.x_size >> 3) * cfont.y_size; j++) {
        ch = cfont.font[pixelIndex];
        for (i = 0; i < 8; i++) {
            if ((ch & (1 << (7 - i))) != 0)
                LCD_Write_DATA16(fch, fcl);
            else
                LCD_Write_DATA16(bch, bcl);
        }
        pixelIndex++;
    }

    LCD_clrXY();
}

// Print string
void LCD_print(const char *st, uint16_t x, uint16_t y) {
    int i = 0;

    while (*st != '\0')
        LCD_printChar(*st++, x + cfont.x_size * i++, y);
}