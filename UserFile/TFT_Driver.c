#include "TFT_Driver.h"

#include <stdio.h>
#include <string.h>

/** \brief at least 240*2 and can be divided wholely by 160*80*2 and larger than 160 * 2*/
#define ST7735_BUF_SIZE (128 * 40)
uint8_t ST7735_Buf[ST7735_BUF_SIZE];
 
 
void ST7735_SendByte(uint8_t dat, ST7735_DCType DC)
{
   ST7735_CS_LOW();
 
   (DC == ST7735_DATA) ? ST7735_DC_HIGH() : ST7735_DC_LOW();
 
   HAL_SPI_Transmit_DMA(ST7735_SPI, &dat, 1);
   while (HAL_SPI_GetState(ST7735_SPI) == HAL_SPI_STATE_BUSY_TX);
 
   ST7735_CS_HIGH();
}
 
void ST7735_SendHalfWord(uint16_t dat)
{
   uint8_t da[2];
   ST7735_CS_LOW();
   ST7735_DC_HIGH();
 
   da[0] = dat >> 8;
   da[1] = dat & 0xFF;
 
   HAL_SPI_Transmit_DMA(ST7735_SPI, da, 2);
   while (HAL_SPI_GetState(ST7735_SPI) == HAL_SPI_STATE_BUSY_TX);
 
   ST7735_CS_HIGH();
}
 
void ST7735_SendMultiByte(uint8_t* dat, uint16_t len)
{
   ST7735_CS_LOW();
   ST7735_DC_HIGH();
   
   HAL_SPI_Transmit_DMA(ST7735_SPI, dat, len);
   while (HAL_SPI_GetState(ST7735_SPI) == HAL_SPI_STATE_BUSY_TX);
 
   ST7735_CS_HIGH();
}
 
 
void ST7735_Init(void)
{
   ST7735_BL_HIGH();
   ST7735_CS_HIGH();
 
   ST7735_RST_HIGH();
   ST7735_RST_LOW();
	HAL_Delay(1);
	ST7735_RST_HIGH();
	HAL_Delay(120);
 
   ST7735_SendByte(0x11, ST7735_CMD);	
   /*Sleep Out When IC is in Sleep In mode, it is necessary to wait 120msec before sending next command 
   because of the stabilization timing for the supply voltages and clock circuits.*/
   HAL_Delay(120);
 
   ST7735_SendByte(0x13, ST7735_CMD);	/*Normal Display Mode On*/
 
   ST7735_SendByte(0x21, ST7735_CMD); /*Display Inversion On*/
 
   ST7735_SendByte(0x26, ST7735_CMD);// Gamma Set
   ST7735_SendByte(0x01, ST7735_DATA);   
 
   ST7735_SendByte(0x29, ST7735_CMD);// Display On
 
 
   ST7735_SendByte(0x2A, ST7735_CMD); //Column Address Set
   ST7735_SendByte(0x00, ST7735_DATA);
   ST7735_SendByte(0x00, ST7735_DATA); //0
   ST7735_SendByte(0x00, ST7735_DATA);
   ST7735_SendByte(0x7f, ST7735_DATA); //79
 
   ST7735_SendByte(0x2B, ST7735_CMD); //Row Address Set
   ST7735_SendByte(0x00, ST7735_DATA);
   ST7735_SendByte(0x00, ST7735_DATA); //0
   ST7735_SendByte(0x00, ST7735_DATA);
   ST7735_SendByte(0x9f, ST7735_DATA); //159
 
   ST7735_SendByte(0x2C, ST7735_CMD);// Memory Write
 
   ST7735_SendByte(0x36, ST7735_CMD); //MX, MY, RGB mode 
#if (ST7735_ROTATION == 0)
      ST7735_SendByte(0x08, ST7735_DATA);
#elif (ST7735_ROTATION == 90)
      ST7735_SendByte(0x68, ST7735_DATA);
#elif (ST7735_ROTATION == 270)
      ST7735_SendByte(0xA8, ST7735_DATA);
#elif (ST7735_ROTATION == 180)
      ST7735_SendByte(0xC8, ST7735_DATA);
#endif
 
   ST7735_SendByte(0x38, ST7735_CMD); //Idle Mode Off
 
   ST7735_SendByte(0x3A, ST7735_CMD);			
   ST7735_SendByte(0x05, ST7735_DATA);   //16-bit/pixel
 
   ST7735_SendByte(0xB1, ST7735_CMD);			//Frame Rate Control (In normal mode/ Full colors)
   ST7735_SendByte(0x02, ST7735_DATA);
   ST7735_SendByte(0x2C, ST7735_DATA); 
   ST7735_SendByte(0x2D, ST7735_DATA); 
 
   ST7735_SendByte(0xC0, ST7735_CMD);			//Power Control 1
   ST7735_SendByte(0x02, ST7735_DATA);
   ST7735_SendByte(0x70, ST7735_DATA);
 
   ST7735_SendByte(0xC1, ST7735_CMD);			//Power Control 2
   ST7735_SendByte(0x05, ST7735_DATA);
 
   ST7735_SendByte(0xC2, ST7735_CMD);			//Power Control 3 (in Normal mode/ Full colors)
   ST7735_SendByte(0x01, ST7735_DATA);
   ST7735_SendByte(0x01, ST7735_DATA);
 
 
   ST7735_SendByte(0xC5, ST7735_CMD);			//VCOM Control 1
   ST7735_SendByte(0x51, ST7735_DATA);       	
   ST7735_SendByte(0x4D, ST7735_DATA);
 
   ST7735_SendByte(0xC7, ST7735_CMD);			//VCOM Offset Control 
   ST7735_SendByte(0xF0, ST7735_DATA);
 
   ST7735_SendByte(0xE0, ST7735_CMD);     
   ST7735_SendByte(0xD0, ST7735_DATA);   
   ST7735_SendByte(0x08, ST7735_DATA);   
   ST7735_SendByte(0x0E, ST7735_DATA);   
   ST7735_SendByte(0x09, ST7735_DATA);   
   ST7735_SendByte(0x09, ST7735_DATA);   
   ST7735_SendByte(0x05, ST7735_DATA);   
   ST7735_SendByte(0x31, ST7735_DATA);   
   ST7735_SendByte(0x33, ST7735_DATA);   
   ST7735_SendByte(0x48, ST7735_DATA);   
   ST7735_SendByte(0x17, ST7735_DATA);   
   ST7735_SendByte(0x14, ST7735_DATA);   
   ST7735_SendByte(0x15, ST7735_DATA);   
   ST7735_SendByte(0x31, ST7735_DATA);   
   ST7735_SendByte(0x34, ST7735_DATA);   
 
   ST7735_SendByte(0xE1, ST7735_CMD);     
   ST7735_SendByte(0xD0, ST7735_DATA);   
   ST7735_SendByte(0x08, ST7735_DATA);   
   ST7735_SendByte(0x0E, ST7735_DATA);   
   ST7735_SendByte(0x09, ST7735_DATA);   
   ST7735_SendByte(0x09, ST7735_DATA);   
   ST7735_SendByte(0x15, ST7735_DATA);   
   ST7735_SendByte(0x31, ST7735_DATA);   
   ST7735_SendByte(0x33, ST7735_DATA);   
   ST7735_SendByte(0x48, ST7735_DATA);   
   ST7735_SendByte(0x17, ST7735_DATA);   
   ST7735_SendByte(0x14, ST7735_DATA);   
   ST7735_SendByte(0x15, ST7735_DATA);   
   ST7735_SendByte(0x31, ST7735_DATA);   
   ST7735_SendByte(0x34, ST7735_DATA);   
 
}
 
 
void ST7735_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
#if (ST7735_ROTATION == 0) || (ST7735_ROTATION == 180)
      x1 += 0x1a;
      x2 += 0x1a;
      y1 += 1;
      y2 += 1;
 
		ST7735_SendByte(0x2a, ST7735_CMD);//列地址设置
		ST7735_SendHalfWord(x1);
		ST7735_SendHalfWord(x2);
		ST7735_SendByte(0x2b, ST7735_CMD);//行地址设置
		ST7735_SendHalfWord(y1);
		ST7735_SendHalfWord(y2);
		ST7735_SendByte(0x2c, ST7735_CMD);//储存器写
#else
      x1 += 1;
      x2 += 1;
      y1 += 0x1a;
      y2 += 0x1a;
 
		ST7735_SendByte(0x2a, ST7735_CMD);//列地址设置
		ST7735_SendHalfWord(x1);
		ST7735_SendHalfWord(x2);
		ST7735_SendByte(0x2b, ST7735_CMD);//行地址设置
		ST7735_SendHalfWord(y1);
		ST7735_SendHalfWord(y2);
		ST7735_SendByte(0x2c, ST7735_CMD);//储存器写
#endif
}
 
void ST7735_Clear(uint16_t color)
{
   uint32_t total_bytes = ST7735_WIDTH * ST7735_HEIGHT * 2; // Total bytes to send
   uint32_t bytes_sent = 0;
   uint16_t i;
   
   ST7735_Address_Set(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);

   // Fill buffer with color data
   for (i = 0; i < ST7735_BUF_SIZE; i += 2)
   {
      ST7735_Buf[i] = color >> 8;
      ST7735_Buf[i + 1] = color & 0xFF;
   }
   
   // Send full buffer chunks
   while (bytes_sent + ST7735_BUF_SIZE <= total_bytes)
   {
      ST7735_SendMultiByte(ST7735_Buf, ST7735_BUF_SIZE);
      bytes_sent += ST7735_BUF_SIZE;
   }
   
   // Send remaining bytes if any
   if (bytes_sent < total_bytes)
   {
      ST7735_SendMultiByte(ST7735_Buf, total_bytes - bytes_sent);
   }
}
void ST7735_ClearRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    // 参数检查
    if (x0 >= ST7735_WIDTH || y0 >= ST7735_HEIGHT || 
        x1 >= ST7735_WIDTH || y1 >= ST7735_HEIGHT ||
        x0 > x1 || y0 > y1)
    {
        // 无效参数，可选择返回错误或处理
        return;
    }
    
    // 计算矩形区域的宽度和高度
    uint16_t rect_width = x1 - x0 + 1;
    uint16_t rect_height = y1 - y0 + 1;
    uint32_t total_pixels = rect_width * rect_height;
    uint32_t total_bytes = total_pixels * 2; // 每个像素2字节
    
    uint32_t bytes_sent = 0;
    uint16_t i;
    
    // 设置要清除的矩形区域地址
    ST7735_Address_Set(x0, y0, x1, y1);
    
    // 填充缓冲区
    for (i = 0; i < ST7735_BUF_SIZE; i += 2)
    {
        ST7735_Buf[i] = color >> 8;
        ST7735_Buf[i + 1] = color & 0xFF;
    }
    
    // 发送完整缓冲区块
    while (bytes_sent + ST7735_BUF_SIZE <= total_bytes)
    {
        ST7735_SendMultiByte(ST7735_Buf, ST7735_BUF_SIZE);
        bytes_sent += ST7735_BUF_SIZE;
    }
    
    // 发送剩余字节
    if (bytes_sent < total_bytes)
    {
        ST7735_SendMultiByte(ST7735_Buf, total_bytes - bytes_sent);
    }
}
void ST7735_DrawPixel(uint16_t x,uint16_t y,uint16_t color)
{
   ST7735_Address_Set(x,y,x,y);//设置光标位置 
   ST7735_SendHalfWord(color);
}
 
void ST7735_DrawHLine(uint16_t xs, uint16_t xe,uint16_t y,uint16_t color)
{
   uint16_t i,j;
   ST7735_Address_Set(xs,y,xe,y);//设置光标位置 
 
   for ( i = 0; i < (xe - xs + 1) * 2; i += 2)
   {
      ST7735_Buf[i] = color >> 8;
      ST7735_Buf[i + 1] = color & 0xFF;
   }
   ST7735_SendMultiByte(ST7735_Buf, (xe - xs + 1) * 2);
}
 
 
 
void ST7735_DrawVLine(uint16_t ys, uint16_t ye, uint16_t x, uint16_t color)
{
   uint16_t i,j;
   ST7735_Address_Set(x,ys,x,ye);//设置光标位置 
 
   for ( i = 0; i < (ye - ys + 1) * 2; i += 2)
   {
      ST7735_Buf[i] = color >> 8;
      ST7735_Buf[i + 1] = color & 0xFF;
   }
   ST7735_SendMultiByte(ST7735_Buf, (ye - ys + 1) * 2);
}
 
void ST7735_FillRect(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color)
{
   uint16_t i,j;
   uint32_t depth;
   depth = (ye - ys + 1) * (xe - xs + 1) * 2;
   ST7735_Address_Set(xs, ys, xe, ye);//设置光标位置 
 
   if (depth < ST7735_BUF_SIZE)
   {
      for ( i = 0; i < depth; i += 2)
      {
         ST7735_Buf[i] = color >> 8;
         ST7735_Buf[i + 1] = color & 0xFF;
      }
      ST7735_SendMultiByte(ST7735_Buf, depth);
   }
   else
   {
      for ( i = 0; i < ST7735_BUF_SIZE; i += 2)
      {
         ST7735_Buf[i] = color >> 8;
         ST7735_Buf[i + 1] = color & 0xFF;
      }
 
      for ( i = 0; i < depth / ST7735_BUF_SIZE; i++)
      {
         ST7735_SendMultiByte(ST7735_Buf, ST7735_BUF_SIZE);
      }
      
      if (depth % ST7735_BUF_SIZE != 0)
      {
         ST7735_SendMultiByte(ST7735_Buf, (depth % ST7735_BUF_SIZE));
      }
   }
}
 
 
void ST7735_DrawBitLine16BPP(uint16_t xs, uint16_t y, uint8_t const * p, uint16_t xsize)
{
   uint16_t i,j;
   ST7735_Address_Set(xs, y, xs + xsize, y);//设置光标位置 
   for ( i = 0; i < xsize * 2; i+=2)
   {
      ST7735_Buf[i + 1] = *(p + i);
      ST7735_Buf[i] = *(p + i + 1);
   }
 
   ST7735_SendMultiByte(ST7735_Buf, xsize * 2);
}
 
 
void ST7735_DrawBitmap(uint16_t xs, uint16_t ys, uint16_t xsize, uint16_t ysize, uint8_t *p)
{
   uint16_t i,j;
   for ( i = 0; i < ysize; i++)
   {
      ST7735_DrawBitLine16BPP(xs, ys + i, p + i * xsize * 2, xsize);
   }
}
 
 
void ST7735_ShowChar(uint16_t x, uint16_t y, uint8_t ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
   uint32_t i, b, j;
   uint8_t data[2] = {0};
   ST7735_Address_Set(x, y, x + font.width - 1, y + font.height - 1);
 
   for (i = 0; i < font.height; i++) 
   {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++) 
      {
			if ((b << j) & 0x8000) 
         {
				data[0] = color >> 8; 
            data[1] = color & 0xFF;
				ST7735_SendMultiByte(data, sizeof(data));
			}
			else 
         {
				data[0] = bgcolor >> 8; 
            data[1] = bgcolor & 0xFF;
				ST7735_SendMultiByte(data, sizeof(data));
			}
		}
	}
}
 
void ST7735_ShowString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
   while (*str) 
   {
		if (x + font.width >= ST7735_WIDTH) 
      {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7735_HEIGHT) 
         {
				break;
			}
			if (*str == ' ') 
         {
				str++;
				continue;
			}
		}
		ST7735_ShowChar(x, y, *str, font, color, bgcolor);
		x += font.width;
		str++;
	}
}


/**/
void ST7735_ShowInt(uint16_t x, uint16_t y, int16_t num, FontDef font, uint16_t color, uint16_t bgcolor)
{
    char str[12]; // 足够存储-2147483648到2147483647
    uint8_t digits = 0;
    int32_t temp = num;
    uint16_t current_x = x;
    
    // 处理负数
    if (num < 0) {
        // 显示负号
        ST7735_ShowChar(current_x, y, '-', font, color, bgcolor);
        current_x += font.width;
        num = -num; // 转为正数处理
    }
    
    // 计算数字位数
    if (num == 0) {
        digits = 1;
    } else {
        temp = num;
        while (temp > 0) {
            digits++;
            temp /= 10;
        }
    }
    
    // 将数字转换为字符串
    snprintf(str, sizeof(str), "%d", num);
    // 逐位显示数字
    for (uint8_t i = 0; i < digits; i++) {
        ST7735_ShowChar(current_x, y, str[i], font, color, bgcolor);
        current_x += font.width;
    }
}

/**
 * @brief 显示浮点数
 * @param x: 起始x坐标
 * @param y: 起始y坐标
 * @param num: 要显示的浮点数
 * @param decimal_places: 小数点后位数 (0-6)
 * @param font: 字体定义
 * @param color: 文字颜色
 * @param bgcolor: 背景颜色
 */
void ST7735_ShowFloat(uint16_t x, uint16_t y, float num, uint8_t decimal_places, FontDef font, uint16_t color, uint16_t bgcolor)
{
    char str[16]; // 足够存储浮点数
    uint16_t current_x = x;
    
    // 参数检查
    if (decimal_places > 6) {
        decimal_places = 6; // 最大支持6位小数
    }
    
    // 处理负数
    if (num < 0) {
        ST7735_ShowChar(current_x, y, '-', font, color, bgcolor);
        current_x += font.width;
        num = -num; // 转为正数处理
    }
    
    // 构建格式化字符串
    char format_str[8];
    snprintf(format_str, sizeof(format_str), "%%.%df", decimal_places);
    
    // 将浮点数转换为字符串
    snprintf(str, sizeof(str), format_str, num);
    
    // 逐位显示字符
    for (uint8_t i = 0; i < strlen(str); i++) {
        ST7735_ShowChar(current_x, y, str[i], font, color, bgcolor);
        current_x += font.width;
    }
}