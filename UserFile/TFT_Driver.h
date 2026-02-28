#ifndef __TFT_DRIVER_H__
#define __TFT_DRIVER_H__		
 

#include "main.h"
#include "TFT_Font.h"
/** \brief ST7735 rotation, can be set to 0 or 90 or 180 or 270 */

extern SPI_HandleTypeDef hspi3;

#define ST7735_ROTATION 90
 
#define ST7735_SPI_DMA
#define ST7735_SPI &hspi3
 
#define ST7735_BUFFER
 
#define RES_GPIO_Port GPIOC
#define RES_Pin GPIO_PIN_11
#define CS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_15
#define DC_GPIO_Port GPIOD
#define DC_Pin GPIO_PIN_2
#define BL_GPIO_Port GPIOB
#define BL_Pin GPIO_PIN_6

 
#if (ST7735_ROTATION == 0) || (ST7735_ROTATION == 180)
   #define ST7735_HEIGHT 160
   #define ST7735_WIDTH 80
#else
   #define ST7735_HEIGHT 80
   #define ST7735_WIDTH 160
#endif
 


#define ST7735_RST_LOW()       HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_RESET)
#define ST7735_RST_HIGH()      HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_SET)
 
#define ST7735_CS_LOW()        HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET)
#define ST7735_CS_HIGH()       HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET)
 
#define ST7735_DC_LOW()        HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_RESET)
#define ST7735_DC_HIGH()       HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_SET)
 
#define ST7735_BL_LOW()        HAL_GPIO_WritePin(BL_GPIO_Port,BL_Pin,GPIO_PIN_RESET)
#define ST7735_BL_HIGH()       HAL_GPIO_WritePin(BL_GPIO_Port,BL_Pin,GPIO_PIN_SET)
 
 
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			    0XFFE0
#define GBLUE			    0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			    0XBC40 //棕色
#define BRRED 			    0XFC07 //棕红色
#define GRAY  			    0X8430 //灰色
 
 
 
 
typedef enum
{
   ST7735_CMD,
   ST7735_DATA,
}ST7735_DCType;
 
 
typedef struct
{
   ST7735_DCType DC;
   uint8_t data;
}ST7735_InitSequenceType;
 
typedef struct {
   uint16_t XSize;
   uint16_t YSize;
   uint16_t BytesPerLine;
   uint16_t BitsPerPixel;
   const uint8_t * pData;
 } GUI_BITMAP;
 
 
void ST7735_Clear(uint16_t Color);
void ST7735_ClearRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ST7735_SendByte(uint8_t dat, ST7735_DCType DC);
void ST7735_SendHalfWord(uint16_t dat);
void ST7735_SendMultiByte(uint8_t* dat, uint16_t len);
void ST7735_Init(void);
 
void ST7735_DrawPixel(uint16_t x,uint16_t y,uint16_t color);
void ST7735_DrawHLine(uint16_t xs, uint16_t xe,uint16_t y,uint16_t color);
void ST7735_DrawVLine(uint16_t ys, uint16_t ye,uint16_t x,uint16_t color);
void ST7735_FillRect(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);
void ST7735_DrawBitmap(uint16_t xs, uint16_t ys, uint16_t xsize, uint16_t ysize, uint8_t *p);
void ST7735_DrawBitLine16BPP(uint16_t xs, uint16_t y, uint8_t const * p, uint16_t xsize);
void ST7735_ShowChar(uint16_t x, uint16_t y, uint8_t ch, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7735_ShowString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);

/*new*/
void ST7735_ShowInt(uint16_t x, uint16_t y, int16_t num, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7735_ShowFloat(uint16_t x, uint16_t y, float num, uint8_t decimal_places, FontDef font, uint16_t color, uint16_t bgcolor);
#endif