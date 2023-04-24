#ifndef __STM32L476R_NUCLEO_Display_H
#define __STM32L476R_NUCLEO_Display_H

#include "stm32l476xx.h"

#define swap(type, i, j)        {type t = i; i = j; j = t;}

#define DISP_X_SIZE             239
#define DISP_Y_SIZE             319
#define MAX_BURST               500


struct current_font {
    uint8_t *font;
    uint8_t x_size;
    uint8_t y_size;
    uint8_t offset;
    uint8_t numchars;
};

// defined in ../Src/fonts.c
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

void Display_GPIO_Init(void);
void LCD_Write_COM(uint8_t cmd);
void LCD_Write_DATA16(uint8_t VH, uint8_t VL);
void LCD_Write_DATA(uint8_t VL);
void Display_init(SPI_TypeDef *SPIx);

void LCD_setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_setColor(uint8_t r, uint8_t g, uint8_t b);
void LCD_setColorBg(uint8_t r, uint8_t g, uint8_t b);
void LCD_clrScr(void);
void LCD_clrXY(void);
void LCD_drawHLine(uint16_t x, uint16_t y, int l);
void LCD_drawVLine(uint16_t x, uint16_t y1, uint16_t y2);
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_fillTriangle(uint16_t x, uint16_t y, int w, int h);
void LCD_setFont(uint8_t *font);
void LCD_printChar(uint8_t c, uint16_t x, uint16_t y);
void LCD_print(const char *st, uint16_t x, uint16_t y);
void LCD_fastFill(void);

#endif