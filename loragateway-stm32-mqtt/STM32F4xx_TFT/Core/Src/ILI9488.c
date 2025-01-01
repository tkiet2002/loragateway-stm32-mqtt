/*
 * ILI9488.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: hydra
 */

#include "ILI9488.h"
#include "stdlib.h"
#include <stdbool.h>
#include <string.h>
extern SPI_HandleTypeDef hspi2;

extern DMA_HandleTypeDef hdma_spi2_tx;


extern uint8_t SPI2_TX_completed_flag;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#define swap(a, b) { int16_t t = a; a = b; b = t; }

uint8_t  tabcolor;
uint8_t mySPCR;

int16_t WIDTH;
int16_t HEIGHT;
 int16_t _width;
int16_t  _height;
int16_t cursor_x;
int16_t cursor_y;
uint16_t textcolor;
uint16_t textbgcolor;
uint8_t textsize_x;
uint8_t textsize_y;
 uint8_t rotation;


void ILI9488_Init(void)
{
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port,TFT_RST_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port,TFT_RST_Pin,GPIO_PIN_SET);
	_width=ILI9488_TFTWIDTH;
	_height=ILI9488_TFTHEIGHT;

//	writecommand(0xE0);
//	writedata(0x00);
//	writedata(0x03);
//	writedata(0x09);
//	writedata(0x08);
//	writedata(0x16);
//	writedata(0x0A);
//	writedata(0x3F);
//	writedata(0x78);
//	writedata(0x4C);
//	writedata(0x09);
//	writedata(0x0A);
//	writedata(0x08);
//	writedata(0x16);
//	writedata(0x1A);
//	writedata(0x0F);
//
//	writecommand(0XE1);
//	writedata(0x00);
//	writedata(0x16);
//	writedata(0x19);
//	writedata(0x03);
//	writedata(0x0F);
//	writedata(0x05);
//	writedata(0x32);
//	writedata(0x45);
//	writedata(0x46);
//	writedata(0x04);
//	writedata(0x0E);
//	writedata(0x0D);
//	writedata(0x35);
//	writedata(0x37);
//	writedata(0x0F);
//
//	writecommand(0XC0);      //Power Control 1
//	writedata(0x17);    //Vreg1out
//	writedata(0x15);    //Verg2out
//
//	writecommand(0xC1);      //Power Control 2
//	writedata(0x41);    //VGH,VGL
//
//	writecommand(0xC5);      //Power Control 3
//	writedata(0x00);
//	writedata(0x12);    //Vcom
//	writedata(0x80);
//
//	writecommand(0x36);      //Memory Access
//	writedata(0x48);
//
//	writecommand(0x36);      // Interface Pixel Format
//	writedata(0x66); 	  //18 bit
//
//	writecommand(0XB0);      // Interface Mode Control
//	writedata(0x80);     			 //SDO NOT USE
//
//	writecommand(0xB1);      //Frame rate
//	writedata(0xA0);    //60Hz
//
//	writecommand(0xB4);      //Display Inversion Control
//	writedata(0x02);    //2-dot
//
//	writecommand(0XB6); //Display Function Control  RGB/MCU Interface Control
//
//	writedata(0x02);    //MCU
//	writedata(0x02);    //Source,Gate scan dieection
//
//	writecommand(0XE9);      // Set Image Functio
//	writedata(0x00);    // Disable 24 bit data
//
//	writecommand(0xF7);      // Adjust Control
//	writedata(0xA9);
//	writedata(0x51);
//	writedata(0x2C);
//	writedata(0x82);    // D7 stream, loose
//
//	writecommand(ILI9488_SLPOUT);    //Exit Sleep
//
//	HAL_Delay(120);
//
//	writecommand(ILI9488_DISPON);    //Display on


//	LCD_Write_COM(0x11);      //ili9481
//	HAL_Delay(20);
//	LCD_Write_COM(0xD0);
//	LCD_Write_DATA(0x07);
//	LCD_Write_DATA(0x42);
//	LCD_Write_DATA(0x18);
//
//	LCD_Write_COM(0xD1);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x07);
//	LCD_Write_DATA(0x10);
//
//	LCD_Write_COM(0xD2);
//	LCD_Write_DATA(0x01);
//	LCD_Write_DATA(0x02);
//
//	LCD_Write_COM(0xC0);
//	LCD_Write_DATA(0x10);
//	LCD_Write_DATA(0x3B);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x02);
//	LCD_Write_DATA(0x11);
//
//	LCD_Write_COM(0xC5);
//	LCD_Write_DATA(0x03);
//
//	LCD_Write_COM(0xC8);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x32);
//	LCD_Write_DATA(0x36);
//	LCD_Write_DATA(0x45);
//	LCD_Write_DATA(0x06);
//	LCD_Write_DATA(0x16);
//	LCD_Write_DATA(0x37);
//	LCD_Write_DATA(0x75);
//	LCD_Write_DATA(0x77);
//	LCD_Write_DATA(0x54);
//	LCD_Write_DATA(0x0C);
//	LCD_Write_DATA(0x00);
//
//	LCD_Write_COM(0x36);
//	LCD_Write_DATA(0x0A);
//
//
//	LCD_Write_COM(0x3A);
//	LCD_Write_DATA(0x55);
//
//	LCD_Write_COM(0x2A);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x01);
//	LCD_Write_DATA(0x3F);
//
//	LCD_Write_COM(0x2B);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x01);
//	LCD_Write_DATA(0xE0);
//	HAL_Delay(120);
//	LCD_Write_COM(0x29);

//	LCD_Write_COM(0x11);		// Sleep OUT//ili9486
//	HAL_Delay(50);
//
//	LCD_Write_COM(0xF2);		// ?????
//	LCD_Write_DATA(0x1C);
//	LCD_Write_DATA(0xA3);
//	LCD_Write_DATA(0x32);
//	LCD_Write_DATA(0x02);
//	LCD_Write_DATA(0xb2);
//	LCD_Write_DATA(0x12);
//	LCD_Write_DATA(0xFF);
//	LCD_Write_DATA(0x12);
//	LCD_Write_DATA(0x00);
//
//	LCD_Write_COM(0xF1);		// ?????
//	LCD_Write_DATA(0x36);
//	LCD_Write_DATA(0xA4);
//
//	LCD_Write_COM(0xF8);		// ?????
//	LCD_Write_DATA(0x21);
//	LCD_Write_DATA(0x04);
//
//	LCD_Write_COM(0xF9);		// ?????
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x08);
//
//	LCD_Write_COM(0xC0);		// Power Control 1
//	LCD_Write_DATA(0x0d);
//	LCD_Write_DATA(0x0d);
//
//	LCD_Write_COM(0xC1);		// Power Control 2
//	LCD_Write_DATA(0x43);
//	LCD_Write_DATA(0x00);
//
//	LCD_Write_COM(0xC2);		// Power Control 3
//	LCD_Write_DATA(0x00);
//
//	LCD_Write_COM(0xC5);		// VCOM Control
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x48);
//
//	LCD_Write_COM(0xB6);		// Display Function Control
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x22);		// 0x42 = Rotate display 180 deg.
//	LCD_Write_DATA(0x3B);
//
//	LCD_Write_COM(0xE0);		// PGAMCTRL (Positive Gamma Control)
//	LCD_Write_DATA(0x0f);
//	LCD_Write_DATA(0x24);
//	LCD_Write_DATA(0x1c);
//	LCD_Write_DATA(0x0a);
//	LCD_Write_DATA(0x0f);
//	LCD_Write_DATA(0x08);
//	LCD_Write_DATA(0x43);
//	LCD_Write_DATA(0x88);
//	LCD_Write_DATA(0x32);
//	LCD_Write_DATA(0x0f);
//	LCD_Write_DATA(0x10);
//	LCD_Write_DATA(0x06);
//	LCD_Write_DATA(0x0f);
//	LCD_Write_DATA(0x07);
//	LCD_Write_DATA(0x00);
//
//	LCD_Write_COM(0xE1);		// NGAMCTRL (Negative Gamma Control)
//	LCD_Write_DATA(0x0F);
//	LCD_Write_DATA(0x38);
//	LCD_Write_DATA(0x30);
//	LCD_Write_DATA(0x09);
//	LCD_Write_DATA(0x0f);
//	LCD_Write_DATA(0x0f);
//	LCD_Write_DATA(0x4e);
//	LCD_Write_DATA(0x77);
//	LCD_Write_DATA(0x3c);
//	LCD_Write_DATA(0x07);
//	LCD_Write_DATA(0x10);
//	LCD_Write_DATA(0x05);
//	LCD_Write_DATA(0x23);
//	LCD_Write_DATA(0x1b);
//	LCD_Write_DATA(0x00);
//
//
//	LCD_Write_COM(0x20);		// Display Inversion OFF
//	LCD_Write_DATA(0x00);//C8
//
//	LCD_Write_COM(0x36);		// Memory Access Control
//	LCD_Write_DATA(0x0A);
//
//	LCD_Write_COM(0x3A);		// Interface Pixel Format
//	LCD_Write_DATA(0x55);
//
//	LCD_Write_COM(0x2A);		// Column Addess Set
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x01);
//	LCD_Write_DATA(0xDF);
//
//	LCD_Write_COM(0x002B);		// Page Address Set
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x01);
//	LCD_Write_DATA(0x3f);
//	HAL_Delay(50);
//	LCD_Write_COM(0x0029);		// Display ON
//	LCD_Write_COM(0x002C);		// Memory Write


//	LCD_Write_COM(0x11);        //ili9488 worked
//	HAL_Delay(20);
//
// 	LCD_Write_COM(0xE0);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x03);
//	LCD_Write_DATA(0x09);
//	LCD_Write_DATA(0x08);
//	LCD_Write_DATA(0x16);
//	LCD_Write_DATA(0x0A);
//	LCD_Write_DATA(0x3F);
//	LCD_Write_DATA(0x78);
//	LCD_Write_DATA(0x4C);
//	LCD_Write_DATA(0x09);
//	LCD_Write_DATA(0x0A);
//	LCD_Write_DATA(0x08);
//	LCD_Write_DATA(0x16);
//	LCD_Write_DATA(0x1A);
//	LCD_Write_DATA(0x0F);
//
//
//	LCD_Write_COM(0XE1);
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x16);
//	LCD_Write_DATA(0x19);
//	LCD_Write_DATA(0x03);
//	LCD_Write_DATA(0x0F);
//	LCD_Write_DATA(0x05);
//	LCD_Write_DATA(0x32);
//	LCD_Write_DATA(0x45);
//	LCD_Write_DATA(0x46);
//	LCD_Write_DATA(0x04);
//	LCD_Write_DATA(0x0E);
//	LCD_Write_DATA(0x0D);
//	LCD_Write_DATA(0x35);
//	LCD_Write_DATA(0x37);
//	LCD_Write_DATA(0x0F);
//
//
//
//
//	LCD_Write_COM(0XC0);      //Power Control 1
//	LCD_Write_DATA(0x17);    //Vreg1out
//	LCD_Write_DATA(0x15);    //Verg2out
//
//	LCD_Write_COM(0xC1);      //Power Control 2
//	LCD_Write_DATA(0x41);    //VGH,VGL
//
//	LCD_Write_COM(0xC5);      //Power Control 3
//	LCD_Write_DATA(0x00);
//	LCD_Write_DATA(0x12);    //Vcom
//	LCD_Write_DATA(0x80);
//
//	LCD_Write_COM(0x36);      //Memory Access
//	LCD_Write_DATA(0x48);
//
//	LCD_Write_COM(0x3A);      // Interface Pixel Format
//	LCD_Write_DATA(0x66); 	  //16 bit
//
//	LCD_Write_COM(0XB0);      // Interface Mode Control
//	LCD_Write_DATA(0x80);
//
//	LCD_Write_COM(0xB1);      //Frame rate
//	LCD_Write_DATA(0xA0);    //60Hz
//
//	LCD_Write_COM(0xB4);      //Display Inversion Control
//	LCD_Write_DATA(0x02);    //2-dot
//
//	LCD_Write_COM(0XB6);      //Display Function Control  RGB/MCU Interface Control
//	LCD_Write_DATA(0x02);    //MCU
//	LCD_Write_DATA(0x02);    //Source,Gate scan dieection
//
//	LCD_Write_COM(0XE9);      // Set Image Functio
//	LCD_Write_DATA(0x00);    // Disable 24 bit data
//
//	LCD_Write_COM(0xF7);      // Adjust Control
//	LCD_Write_DATA(0xA9);
//	LCD_Write_DATA(0x51);
//	LCD_Write_DATA(0x2C);
//	LCD_Write_DATA(0x82);    // D7 stream, loose
//
//	LCD_Write_COM(0x11); //Sleep out
//	HAL_Delay(120);
//	LCD_Write_COM(0x29);


//	  writeCommand(EXIT_SLEEP_MODE); //exit sleep mode
//	  HAL_Delay(80);
//
//	  writeCommand(POWER_SETTING); //power setting
//	    writeData8(0x07U);
//	    writeData8(0x42U);
//	    writeData8(0x18U);
//
//	  writeCommand(VCOM_CONTROL); //VCOM control
//	    writeData8(0x00U);
//	    writeData8(0x07U);
//	    writeData8(0x10U);
//
//	  writeCommand(POWER_SETTING_NORMAL); //power setting for normal mode
//	    writeData8(0x01U);
//	    writeData8(0x02U); //Fosc setting
//
//	  writeCommand(PANEL_DRIVE_SETTING); //panel driving setting - 5 parameters
//	    writeData8(0x10U); //REV=1, SM=0, GS=0 - grayscale inversion enabled : will invert colors
//	    //writeData8(0x00); //REV=0, SM=0, GS=0 - no color inversion
//
//	    writeData8(0x3BU); //NL[5] - max lines
//	    writeData8(0x00U); //SCN - scanning start position
//	    writeData8(0x0U); //PTV=1 (halts vcom)
//	    writeData8(0x02U); //NDL (non-display area o/p level), PTS[3]
//
//	    writeData8(0x1U); //PTG=0 (interval scan), ISC[3]=0001 (3 frames)
//	    // writeData8(0x11); //PTG=1 (interval scan), ISC[3]=0001 (3 frames)
//	    // writeData8(0x1F); //PTG=1 (interval scan), ISC[3]=1111 (31 frames)
//	    // writeData8(0x01); //PTG=0 (normal scan), ISC[3]=0002 (3 frames)
//
//	  writeCommand(FRAME_RATE_CONTROL); //frame rate and inversion control - 1 parameter
//	    //writeData8(0x03); //72FPS (default) - this had backlight flickering
//	    writeData8(0x00U); //125 FPS (max) - this has no flickering
//
//	  writeCommand(FRAME_MEMORY_ACCESS_SETTING); //frame memeory access and interface setting
//	    writeData8(0x0U); //extra data is ignored
//	    writeData8(0x0U); //all defaults
//	    writeData8(0x0U);
//	    writeData8(0x0U);
//
//	  writeCommand(SET_TEAR_OFF); //tearing compensation
//	    // writeData8(0x0);
//
//	  writeCommand(TIMING_SETTING_NORMAL); //display timing for normal mode
//	    writeData8(0x10U); //BC=0 (frame inversion waveform)
//	    writeData8(0x10U); //line period is 16 clocks
//	    writeData8(0x22U); //front and back porch lines = 2
//
//	  writeCommand(GAMMA_SETTING); //gamma setting
//	    writeData8(0x00U);
//	    writeData8(0x32U);
//	    writeData8(0x36U);
//	    writeData8(0x45U);
//	    writeData8(0x06U);
//	    writeData8(0x16U);
//	    writeData8(0x37U);
//	    writeData8(0x75U);
//	    writeData8(0x77U);
//	    writeData8(0x54U);
//	    writeData8(0x0CU);
//	    writeData8(0x00U);
//
//	  writeCommand(SET_ADDRESS_MODE); //set address mode - 1 parameter
//	    writeData8(0x0AU); //BGR, horizontal flip
//
//	  writeCommand(ENTER_NORMAL_MODE); //eneter normal mode - no param
//	  writeData8(0x80U);
//
//	  writeCommand(SET_PIXEL_FORMAT); //set pixel format - 1 parameter
//	    writeData8(0x55U); //16-bit per pixel
//	    // writeData(0x66); //18-bit per pixel
//
////	  setAddrWindow(0, 0, _width, _height);
//	  writeCommand(0x2A); //set column address
//	    writeData8(0x00);
//	    writeData8(0x00);
//	    writeData8(0x01);
//	    writeData8(0x3F);
//
//	  writeCommand(0x2B); //set page address
//	    writeData8(0x00);
//	    writeData8(0x00);
//	    writeData8(0x01);
//	    writeData8(0xDF);
//	    HAL_Delay(120);
//
//	  writeCommand(SET_DISPLAY_ON); //set display on



//	ILI9488WriteReg( 0xF7);
//	ILI9488WriteData( 0xA9);
//	ILI9488WriteData( 0x51);
//	ILI9488WriteData( 0x2C);
//	ILI9488WriteData( 0x82);
//
//	ILI9488WriteReg( 0xC0);
//	ILI9488WriteData( 0x11);
//	ILI9488WriteData( 0x09);
//
//	ILI9488WriteReg( 0xC1);
//	ILI9488WriteData( 0x41);
//
//	ILI9488WriteReg( 0xC5);
//	ILI9488WriteData( 0x00);
//	ILI9488WriteData( 0x2A);
//	ILI9488WriteData( 0x80);
//
//	ILI9488WriteReg(0XB0);      // Interface Mode Control
//	ILI9488WriteData(0x00);     			 //SDO NOT USE
//
//	ILI9488WriteReg( 0xB1);
//	ILI9488WriteData( 0xB0);
////	ILI9488WriteData( 0x11);
//
//	ILI9488WriteReg( 0xB4);
//	ILI9488WriteData( 0x02);
//
//	ILI9488WriteReg( 0xB6);
//	ILI9488WriteData( 0x02);
//	ILI9488WriteData( 0x22);
//	ILI9488WriteData( 0x3B);
//
//	ILI9488WriteReg( 0xB7);
//	ILI9488WriteData( 0xC6);
//
//	ILI9488WriteReg( 0xBE);
//	ILI9488WriteData( 0x00);
//	ILI9488WriteData( 0x04);
//
//	ILI9488WriteReg( 0xE9);
//	ILI9488WriteData( 0x00);
//
//	ILI9488WriteReg( 0x36);// Interface Pixel Format
//	ILI9488WriteData( 0x08);
//
//	ILI9488WriteReg( 0x3A);
//	ILI9488WriteData( 0x55);
//
//	ILI9488WriteReg( 0xE0);
//	ILI9488WriteData( 0x00);
//	ILI9488WriteData( 0x07);
//	ILI9488WriteData( 0x12);
//	ILI9488WriteData( 0x0B);
//	ILI9488WriteData( 0x18);
//	ILI9488WriteData( 0x0B);
//	ILI9488WriteData( 0x3F);
//	ILI9488WriteData( 0x9B);
//	ILI9488WriteData( 0x4B);
//	ILI9488WriteData( 0x0B);
//	ILI9488WriteData( 0x0F);
//	ILI9488WriteData( 0x0B);
//	ILI9488WriteData( 0x15);
//	ILI9488WriteData( 0x17);
//	ILI9488WriteData( 0x0F);
//
//	ILI9488WriteReg( 0xE1);
//	ILI9488WriteData( 0x00);
//	ILI9488WriteData( 0x16);
//	ILI9488WriteData( 0x1B);
//	ILI9488WriteData( 0x02);
//	ILI9488WriteData( 0x0F);
//	ILI9488WriteData( 0x06);
//	ILI9488WriteData( 0x34);
//	ILI9488WriteData( 0x46);
//	ILI9488WriteData( 0x48);
//	ILI9488WriteData( 0x04);
//	ILI9488WriteData( 0x0D);
//	ILI9488WriteData( 0x0D);
//	ILI9488WriteData( 0x35);
//	ILI9488WriteData( 0x36);
//	ILI9488WriteData( 0x0F);
//
//	ILI9488WriteReg( 0x11);
//  	HAL_Delay(120);
//	ILI9488WriteReg( 0x29);


	lcd_write_command(0xE0); //P-Gamma
	lcd_write_data(0x00);
	lcd_write_data(0x13);
	lcd_write_data(0x18);
	lcd_write_data(0x04);
	lcd_write_data(0x0F);
	lcd_write_data(0x06);
	lcd_write_data(0x3A);
	lcd_write_data(0x56);
	lcd_write_data(0x4D);
	lcd_write_data(0x03);
	lcd_write_data(0x0A);
	lcd_write_data(0x06);
	lcd_write_data(0x30);
	lcd_write_data(0x3E);
	lcd_write_data(0x0F);
	lcd_write_command(0XE1); //N-Gamma
	lcd_write_data(0x00);
	lcd_write_data(0x13);
	lcd_write_data(0x18);
	lcd_write_data(0x01);
	lcd_write_data(0x11);
	lcd_write_data(0x06);
	lcd_write_data(0x38);
	lcd_write_data(0x34);
	lcd_write_data(0x4D);
	lcd_write_data(0x06);
	lcd_write_data(0x0D);
	lcd_write_data(0x0B);
	lcd_write_data(0x31);
	lcd_write_data(0x37);
	lcd_write_data(0x0F);
	lcd_write_command(0xC0);
	lcd_write_data(0x18);
	lcd_write_data(0x16);
	lcd_write_command(0xC1);
	lcd_write_data(0x45);
	lcd_write_command(0xC5); //VCOM
	lcd_write_data(0x00);
	lcd_write_data(0x63);
	lcd_write_data(0x01);

	lcd_write_command(0x36); //RAM address mode
	//0xF8 and 0x3C are landscape mode. 0x5C and 0x9C for portrait mode.
//	if(LANDSCAPE)
		lcd_write_data(0xF8);
//	else
//		lcd_write_data(0x5C);

	lcd_write_command(0x3A); //Interface Mode Control
	lcd_write_data(0x66); //16-bit serial mode
	lcd_write_command(0xB0); //Interface Mode Control
	lcd_write_data(0x80); //SDO not in use
	lcd_write_command(0xB1); //Frame rate 70HZ
	lcd_write_data(0x00); //
	lcd_write_data(0x10);
	lcd_write_command(0xB4);
	lcd_write_data(0x02);

	lcd_write_command(0xB6); //RGB/MCU Interface Control
	lcd_write_data(0x02);
	//lcd_write_data(0x22);

	lcd_write_command(0xE9);
	lcd_write_data(0x00);
	lcd_write_command(0xF7);
	lcd_write_data(0xA9);
	lcd_write_data(0x51);
	lcd_write_data(0x2C);
	lcd_write_data(0x82);
	lcd_write_command(0x11);
	HAL_Delay(120);
	lcd_write_command(0x21);


	HAL_Delay(120);
	lcd_write_command(0x29);


}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	writecommand(ILI9488_CASET); // Column addr set
	writedata(x0 >> 8);
	writedata(x0 & 0xFF);     // XSTART
	writedata(x1 >> 8);
	writedata(x1 & 0xFF);     // XEND
	writecommand(ILI9488_PASET); // Row addr set
	writedata(y0 >> 8);
	writedata(y0 & 0xff);     // YSTART
	writedata(y1 >> 8);
	writedata(y1 & 0xff);     // YEND
	writecommand(ILI9488_RAMWR); // write to RAM
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
}
void setScrollArea(uint16_t topFixedArea, uint16_t bottomFixedArea)
{
	writecommand(0x33); // Vertical scroll definition
	writedata(topFixedArea >> 8);
	writedata(topFixedArea);
	writedata((_height - topFixedArea - bottomFixedArea) >> 8);
	writedata(_height - topFixedArea - bottomFixedArea);
	writedata(bottomFixedArea >> 8);
	writedata(bottomFixedArea);
}
void scroll(uint16_t pixels)
{
	writecommand(0x37); // Vertical scrolling start address
	writedata(pixels >> 8);
	writedata(pixels);
}
void pushColor(uint16_t color)
{
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	write16BitColor(color);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);
}
void pushColors(uint16_t *data, uint8_t len, uint8_t first)
{
	uint16_t color;
	uint8_t buff[len * 3 + 1];
	uint16_t count = 0;
	uint8_t lencount = len;


	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	if (first == 1) {
		HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	}
	while (lencount--)
	{
		color = *data++;
		buff[count] = (((color & 0xF800) >> 11) * 255) / 31;
		count++;
		buff[count] = (((color & 0x07E0) >> 5) * 255) / 63;
		count++;
		buff[count] = ((color & 0x001F) * 255) / 31;
		count++;
	}
	HAL_SPI_Transmit(&hspi2, buff, len * 3, 100);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}
void drawImage(const uint8_t* img, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{

	if ((x >= _width) || (y >= _height))
		return;
	if ((x + w - 1) >= _width)
		w = _width - x;
	if ((y + h - 1) >= _height)
		h = _height - y;
	setAddrWindow(x, y, x + w - 1, y + h - 1);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	uint8_t linebuff[w * 3 + 1];
	uint32_t count = 0;
	for (uint16_t i = 0; i < h; i++) {
		uint16_t pixcount = 0;
		for (uint16_t o = 0; o < w; o++) {
			uint8_t b1 = img[count];
			count++;
			uint8_t b2 = img[count];
			count++;
			uint16_t color = b1 << 8 | b2;
			linebuff[pixcount] = (((color & 0xF800) >> 11) * 255)
					/ 31;
			pixcount++;
			linebuff[pixcount] = (((color & 0x07E0) >> 5) * 255)
					/ 63;
			pixcount++;
			linebuff[pixcount] = ((color & 0x001F) * 255) / 31;
			pixcount++;
		}
//		HAL_SPI_Transmit(_spi->getHandler(), linebuff, w * 3, 100);
		HAL_SPI_Transmit(&hspi2, linebuff, w * 3, 100);

	}

	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);
}
void fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}
void drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
		return;

	setAddrWindow(x, y, x + 1, y + 1);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	write16BitColor(color);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{

	if ((x >= _width) || (y >= _height))
		return;

	if ((y + h - 1) >= _height)
		h = _height - y;

	setAddrWindow(x, y, x, y + h - 1);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	while (h--) {
		write16BitColor(color);
	}
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}
void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{

	if ((x >= _width) || (y >= _height))
		return;
	if ((x + w - 1) >= _width)
		w = _width - x;
	setAddrWindow(x, y, x + w - 1, y);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	while (w--)
	{
		write16BitColor(color);
	}
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);
}
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color)
{
	 if (x0 == x1) {
	    if (y0 > y1)
	      _swap_int16_t(y0, y1);
	    drawFastVLine(x0, y0, y1 - y0 + 1, color);
	  } else if (y0 == y1) {
	    if (x0 > x1)
	      _swap_int16_t(x0, x1);
	    drawFastHLine(x0, y0, x1 - x0 + 1, color);
	  } else {

	    writeLine(x0, y0, x1, y1, color);

	  }

}
void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color) {

  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	if ((x >= _width) || (y >= _height))
		return;
	if ((x + w - 1) >= _width)
		w = _width - x;
	if ((y + h - 1) >= _height)
		h = _height - y;

	setAddrWindow(x, y, x + w - 1, y + h - 1);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	for (y = h; y > 0; y--) {
		for (x = w; x > 0; x--) {

			write16BitColor(color);
		}
	}

	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

//		for (y = h; y > 0; y--) {
//			for (x = w; x > 0; x--) {
//
//				write16BitColor(ILI9488_DARKGREY);
//			}
//		}

//		HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}
void setRotation(uint8_t r)
{

	writecommand(ILI9488_MADCTL);
	rotation = r % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		writedata(MADCTL_MX | MADCTL_BGR);
		_width = ILI9488_TFTWIDTH;
		_height = ILI9488_TFTHEIGHT;
		break;
	case 1:
		writedata(MADCTL_MV | MADCTL_BGR);
		_width = ILI9488_TFTHEIGHT;
		_height = ILI9488_TFTWIDTH;
		break;
	case 2:
		writedata(MADCTL_MY | MADCTL_BGR);
		_width = ILI9488_TFTWIDTH;
		_height = ILI9488_TFTHEIGHT;
		break;
	case 3:
		writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width = ILI9488_TFTHEIGHT;
		_height = ILI9488_TFTWIDTH;
		break;
	}

}
void invertDisplay(uint8_t i)
{

	  writecommand(i ? ILI9488_INVON : ILI9488_INVOFF);

}
uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
void spiwrite(uint8_t data)
{
	HAL_SPI_Transmit(&hspi2, &data, 1, 1);

}
void writecommand(uint8_t c)
{

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_RESET);


	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &c, 1, 1);


	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}


void lcd_write_command(uint8_t c)
{

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_RESET);


	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &c, 1, 1);


	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}


void write16BitColor(uint16_t color)
{

	  uint8_t r = (color & 0xF800) >> 11;
	  uint8_t g = (color & 0x07E0) >> 5;
	  uint8_t b = color & 0x001F;

	  r = (r * 255) / 31;
	  g = (g * 255) / 63;
	  b = (b * 255) / 31;


	  HAL_SPI_Transmit(&hspi2, &r, 1, 1);
	  HAL_SPI_Transmit(&hspi2, &g, 1, 1);
	  HAL_SPI_Transmit(&hspi2, &b, 1, 1);


}
void writedata(uint8_t d)
{


	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &d, 1, 1);
	//HAL_SPI_Transmit(_spi.getHandler(), &tmp,1,100);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}

void lcd_write_data(uint8_t d)
{


	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &d, 1, 1);
	//HAL_SPI_Transmit(_spi.getHandler(), &tmp,1,100);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);

}

void testLines(uint8_t color)
{

//	unsigned long start, t;
	int x1, y1, x2, y2, w = _width, h = _height;
	fillScreen(BLACK);

	x1 = y1 = 0;
	y2 = h - 1;

	for (x2 = 0; x2 < w; x2 += 6)
		drawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		drawLine(x1, y1, x2, y2, color);
	fillScreen(BLACK);

	x1 = w - 1;
	y1 = 0;
	y2 = h - 1;

	for (x2 = 0; x2 < w; x2 += 6)
		drawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		drawLine(x1, y1, x2, y2, color);

	fillScreen(BLACK);

	x1 = 0;
	y1 = h - 1;
	y2 = 0;

	for (x2 = 0; x2 < w; x2 += 6)
		drawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		drawLine(x1, y1, x2, y2, color);

	fillScreen(BLACK);

	x1 = w - 1;
	y1 = h - 1;
	y2 = 0;

	for (x2 = 0; x2 < w; x2 += 6)
		drawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		drawLine(x1, y1, x2, y2, color);
}
void sendasIO(uint8_t d)
{
	for (uint8_t bit = 0x80; bit; bit >>= 1) {

		if (d & bit) {
			//_mosi->setHigh();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		} else {
			//_mosi->setLow();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		//_sck->setHigh();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//HAL_Delay(4);
		//__ASM volatile ("NOP");

		//_sck->setLow();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
/*void ILI9488::commandList(uint8_t *addr)
 {
	uint8_t numCommands, numArgs;
	uint16_t ms;

	numCommands = pgm_read_byte(addr++);
	while (numCommands--) {
		writecommand(pgm_read_byte(addr++));
		numArgs = pgm_read_byte(addr++);
		ms = numArgs & 0x80;
		numArgs &= ~0x80;
		while (numArgs--) {
			writedata(pgm_read_byte(addr++));
		}

		if (ms) {
			ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
			if (ms == 255)
				ms = 500;     // If 255, delay for 500 ms
			delay(ms);
		}
	}

}*/


void ILI9341_Draw_Colour(uint16_t Colour)
{
//SENDS COLOUR



	  uint8_t r = (Colour & 0xF800) >> 11;
	  uint8_t g = (Colour & 0x07E0) >> 5;
	  uint8_t b = Colour & 0x001F;

	  r = (r * 255) / 31;
	  g = (g * 255) / 63;
	  b = (b * 255) / 31;

		unsigned char TempBuffer[3] =
		{ r, g ,b };

		for (uint32_t j = 0; j < 480*320; j++)
		{
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);

	SPI2_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(&hspi2, TempBuffer, 3);
	while (SPI2_TX_completed_flag == 0)

	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
		}
}

//INTERNAL FUNCTION OF LIBRARY
/*Sends block colour information to LCD*/
void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size)
{
//SENDS COLOUR
	uint32_t Buffer_Size = 0;
	if ((Size * 2) < BURST_MAX_SIZE)
	{
		Buffer_Size = Size;
	}
	else
	{
		Buffer_Size = BURST_MAX_SIZE;
	}

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);

//	unsigned char MSB_color = Colour >> 8;

	  uint8_t r = (Colour & 0xF800) >> 11;
	  uint8_t g = (Colour & 0x07E0) >> 5;
	  uint8_t b = Colour & 0x001F;

	  r = (r * 255) / 31;
	  g = (g * 255) / 63;
	  b = (b * 255) / 31;

	unsigned char burst_buffer[Buffer_Size];
	for (uint32_t j = 0; j < Buffer_Size; j += 3)
	{
		burst_buffer[j] = r;
		burst_buffer[j + 1] = g;
		burst_buffer[j + 2] = b;
	}

	uint32_t Sending_Size = Size * 3;
	uint32_t Sending_in_Block = Sending_Size / Buffer_Size;
	uint32_t Remainder_from_block = Sending_Size % Buffer_Size;

	if (Sending_in_Block != 0)
	{
		for (uint32_t j = 0; j < (Sending_in_Block); j++)
		{
			SPI2_TX_completed_flag = 0;
			HAL_SPI_Transmit_DMA(&hspi2, (unsigned char*) burst_buffer, Buffer_Size);
			while (SPI2_TX_completed_flag == 0);
		}
	}

//REMAINDER!

	if (Remainder_from_block > 0)
	{
		SPI2_TX_completed_flag = 0;
		HAL_SPI_Transmit_DMA(&hspi2, (unsigned char*) burst_buffer, Remainder_from_block);
		while (SPI2_TX_completed_flag == 0);
	}
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
}

//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
void ILI9341_Fill_Screen(uint16_t Colour)
{
//	LCD_Address_Set(xsta,ysta+OFFSET_Y,xend-1,yend-1+OFFSET_Y);/
	setAddrWindow(0, 0, ILI9488_TFTHEIGHT-1, ILI9488_TFTWIDTH-1);
	ILI9341_Draw_Colour_Burst(Colour, ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT);
}


void ILI9341_Fill_Screen1(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t Colour)
{
//	LCD_Address_Set(xsta,ysta+OFFSET_Y,xend-1,yend-1+OFFSET_Y);/
	setAddrWindow(x, y, h-1, w-1);
	ILI9341_Draw_Colour_Burst(Colour, w * h);
}

//static void WriteDataDMA(const void *data, uint16_t length)
//{
//	txComplete = false;
//	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, length);
//}
//
//static void WaitForDMAWriteComplete(void)
//{
//	while (txComplete == false)
//	{
//	}
//}
//
//void ILI9341FilledRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, colour_t colour)
//{
//	colour_t dmaBuffer[DMA_BUFFER_SIZE];
//	uint8_t i;
//	uint32_t totalBytesToWrite;
//	uint32_t bytesToWriteThisTime;
//
//	for (i = 0U; i < DMA_BUFFER_SIZE; i++)
//	{
//		dmaBuffer[i] = __builtin_bswap16(colour);
//	}
//
//	setAddrWindow(x, y, x + ILI9488_TFTWIDTH - 1U, y + ILI9488_TFTHEIGHT - 1U);
//	totalBytesToWrite = (uint32_t)width * (uint32_t)height * (uint32_t)sizeof(colour_t);
//	bytesToWriteThisTime = DMA_BUFFER_SIZE * (uint16_t)sizeof(colour_t);
//
//	while (totalBytesToWrite > 0UL)
//	{
//		if (totalBytesToWrite < bytesToWriteThisTime)
//		{
//			bytesToWriteThisTime = totalBytesToWrite;
//		}
//		totalBytesToWrite -= bytesToWriteThisTime;
//
//		WriteDataDMA(&dmaBuffer, bytesToWriteThisTime);
//		WaitForDMAWriteComplete();
//	}
//}
//
//static void filledRectangleClip(int16_t x, int16_t y, uint16_t width, uint16_t height, colour_t colour)
//{
//	int16_t x_end;
//	int16_t y_end;
//
//	if (x > (int16_t)ILI9488_TFTWIDTH - 1 || y > (int16_t)ILI9488_TFTHEIGHT - 1)
//	{
//		return;
//	}
//
//	if (x + (int16_t)width < 0 || y + (int16_t)height < 0)
//	{
//		return;
//	}
//
//	if (x < 0)
//	{
//		width -= (uint16_t)-x;
//		x = 0;
//	}
//
//	if (y < 0)
//	{
//		height -= (uint16_t)-y;
//		y = 0;
//	}
//
//	x_end = x + (int16_t)width - 1;
//	if (x_end >(int16_t)ILI9488_TFTWIDTH - 1)
//	{
//		x_end = (int16_t)ILI9488_TFTWIDTH - 1;
//	}
//
//	y_end = y + (int16_t)height - 1;
//	if (y_end > (int16_t)ILI9488_TFTHEIGHT - 1)
//	{
//		y_end = (int16_t)ILI9488_TFTHEIGHT - 1;
//	}
//
//	// clipped sizes
//	width = (uint16_t)(x_end - x + 1);
//	height = (uint16_t)(y_end - y + 1);
//
//	ILI9341FilledRectangle((uint16_t)x, (uint16_t)y, width, height, colour);
//}
//
//
//void GraphicsClear(colour_t colour)
//{
//	filledRectangleClip(0U, 0U, (uint16_t)ILI9488_TFTWIDTH, (uint16_t)ILI9488_TFTHEIGHT, colour);
//}
//



void LCD_Char(int16_t x, int16_t y, const GFXglyph *glyph, const GFXfont *font, uint8_t size, uint32_t color24)
{
	uint8_t  *bitmap = font -> bitmap;
	uint16_t bo = glyph -> bitmapOffset;
	uint8_t bits = 0, bit = 0;
	uint16_t set_pixels = 0;
	uint8_t  cur_x, cur_y;
	for (cur_y = 0; cur_y < glyph -> height; cur_y++)
	{
		for (cur_x = 0; cur_x < glyph -> width; cur_x++)
		{
			if (bit == 0)
			{
				bits = (*(const unsigned char *)(&bitmap[bo++]));
				bit  = 0x80;
			}
			if (bits & bit) set_pixels++;
			else if (set_pixels > 0)
			{
				fillRect(x + (glyph -> xOffset + cur_x - set_pixels) * size, y + (glyph -> yOffset + cur_y) * size, size * set_pixels, size, color24);
				set_pixels = 0;
			}
			bit >>= 1;
		}
		if (set_pixels > 0)
		{
			fillRect(x + (glyph -> xOffset + cur_x-set_pixels) * size, y + (glyph -> yOffset + cur_y) * size, size * set_pixels, size, color24);
			set_pixels = 0;
		}
	}
}

void LCD_Font(uint16_t x, uint16_t y, const char *text, const GFXfont *p_font, uint8_t size, uint32_t color24)
{
	int16_t cursor_x = x;
	int16_t cursor_y = y;
	GFXfont font;
	memcpy(&font, p_font, sizeof(GFXfont));
	for (uint16_t text_pos = 0; text_pos < strlen(text); text_pos++)
	{
		char c = text[text_pos];
		if (c == '\n')
		{
			cursor_x = x;
			cursor_y += font.yAdvance * size;
		}
		else if (c >= font.first && c <= font.last && c != '\r')
		{
			GFXglyph glyph;
			memcpy(&glyph, &font.glyph[c - font.first], sizeof(GFXglyph));
			LCD_Char(cursor_x, cursor_y, &glyph, &font, size, color24);
			cursor_x += glyph.xAdvance * size;
		}
	}
}







void fillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
  int32_t
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }
}


uint16_t rainbow(uint16_t value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

	uint16_t red = 0; // Red is the top 5 bits of a 16 bit colour value
	uint16_t green = 0;// Green is the middle 6 bits
	uint16_t blue = 0; // Blue is the bottom 5 bits

	uint16_t quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}


//int ringMeter(int value, int vmin, int vmax, int x, int y, int r,int w, uint16_t scheme){
//  // Minimum value of r is about 52 before value text intrudes on ring
//  // drawing the text first is an option
//  x += r; y += r;   // Calculate coords of centre of ring
////  int w = r / 8;    // Width of outer ring is 1/4 of radius
//  int angle = 150;  // Half the sweep angle of meter (300 degrees)
////  int text_colour = 0; // To hold the text colour
//  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
//  uint16_t seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
//  uint16_t inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring
//
//  // Draw colour blocks every inc degrees
//  for (int i = -angle; i < angle; i += inc) {
//    // Choose colour from scheme
//    int colour = 0;
//    switch (scheme) {
//      case 0: colour = RED; break; // Fixed colour
//      case 1: colour = GREEN; break; // Fixed colour
//      case 2: colour = BLUE; break; // Fixed colour
//      case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
//      case 4: colour = rainbow(map(i, -angle, angle, 63, 127)); break; // Green to red (high temperature etc)
//      case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
//      case 8: colour = YELLOW; break;
//      default: colour = BLUE; break; // Fixed colour
//    }
//
//    // Calculate pair of coordinates for segment start
//    float sx = cos((i - 90) * 0.0174532925);
//    float sy = sin((i - 90) * 0.0174532925);
//    uint16_t x0 = sx * (r - w) + x;
//    uint16_t y0 = sy * (r - w) + y;
//    uint16_t x1 = sx * r + x;
//    uint16_t y1 = sy * r + y;
//
//    // Calculate pair of coordinates for segment end
//    float sx2 = cos((i + seg - 90) * 0.0174532925);
//    float sy2 = sin((i + seg - 90) * 0.0174532925);
//    int x2 = sx2 * (r - w) + x;
//    int y2 = sy2 * (r - w) + y;
//    int x3 = sx2 * r + x;
//    int y3 = sy2 * r + y;
//
//    if (i < v) { // Fill in coloured segments with 2 triangles
////      my_lcd.Set_Draw_color(colour);
////      my_lcd.Fill_Triangle(x0, y0, x1, y1, x2, y2);
////      my_lcd.Fill_Triangle(x1, y1, x2, y2, x3, y3);
////      text_colour = colour; // Save the last colour drawn
//
//    	fillTriangle(x0, y0, x1, y1, x2, y2, colour);
//    	fillTriangle(x1, y1, x2, y2, x3, y3, colour);
//
//    }
//    else // Fill in blank segments
//    {
////      my_lcd.Set_Draw_color(GRAY);
////      my_lcd.Fill_Triangle(x0, y0, x1, y1, x2, y2);
////      my_lcd.Fill_Triangle(x1, y1, x2, y2, x3, y3);
//
//    	fillTriangle(x0, y0, x1, y1, x2, y2, ILI9488_BLUE);
//    	fillTriangle(x1, y1, x2, y2, x3, y3, ILI9488_BLUE);
//
//    }
//
//
//  }
//
//  return x + r;
//}
//




int ringMeter1(int value, int vmin, int vmax, int x, int y, int r,int w,uint16_t bcolor, uint16_t scheme){
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  x += r; y += r;   // Calculate coords of centre of ring
//  int w = r / 8;    // Width of outer ring is 1/4 of radius
  int angle = 150;  // Half the sweep angle of meter (300 degrees)
//  int text_colour = 0; // To hold the text colour
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  uint16_t seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
  uint16_t inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring

  // Draw colour blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) {
    // Choose colour from scheme
    int colour = 0;
    switch (scheme) {
      case 0: colour = RED; break; // Fixed colour
      case 1: colour = GREEN; break; // Fixed colour
      case 2: colour = BLUE; break; // Fixed colour
      case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
      case 4: colour = rainbow(map(i, -angle, angle, 63, 127)); break; // Green to red (high temperature etc)
      case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
      case 6: colour = BLACK; break;
      case 7: colour = PINK; break;
      default: colour = BLUE; break; // Fixed colour
    }

    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
//      my_lcd.Set_Draw_color(colour);
//      my_lcd.Fill_Triangle(x0, y0, x1, y1, x2, y2);
//      my_lcd.Fill_Triangle(x1, y1, x2, y2, x3, y3);
//      text_colour = colour; // Save the last colour drawn

    	fillTriangle(x0, y0, x1, y1, x2, y2, colour);
    	fillTriangle(x1, y1, x2, y2, x3, y3, colour);

    }
    else // Fill in blank segments
    {
//      my_lcd.Set_Draw_color(GRAY);
//      my_lcd.Fill_Triangle(x0, y0, x1, y1, x2, y2);
//      my_lcd.Fill_Triangle(x1, y1, x2, y2, x3, y3);

    	fillTriangle(x0, y0, x1, y1, x2, y2, bcolor);
    	fillTriangle(x1, y1, x2, y2, x3, y3, bcolor);

    }


  }

  return x + r;
}




void LCD_Circle_Helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint8_t size, uint32_t color24)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		if (cornername & 0x4) {
			fillRect(x0 + x, y0 + y, size, size, color24);
			fillRect(x0 + y, y0 + x, size, size, color24);
		}
		if (cornername & 0x2) {
			fillRect(x0 + x, y0 - y, size, size, color24);
			fillRect(x0 + y, y0 - x, size, size, color24);
		}
		if (cornername & 0x8) {
			fillRect(x0 - y, y0 + x, size, size, color24);
			fillRect(x0 - x, y0 + y, size, size, color24);
		}
		if (cornername & 0x1) {
			fillRect(x0 - y, y0 - x, size, size, color24);
			fillRect(x0 - x, y0 - y, size, size, color24);
		}
	}
}

void LCD_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint32_t color24)
{
	int deltaX = abs(x2 - x1);
	int deltaY = abs(y2 - y1);
	int signX = x1 < x2 ? 1 : -1;
	int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	int error2 = 0;
	for (;;)
	{
		fillRect(x1, y1, size, size, color24);
		if (x1 == x2 && y1 == y2)
		break;
		error2 = error * 2;
		if (error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		if (error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
}


void LCD_Rect_Round(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t r, uint8_t size, uint32_t color24)
{
	LCD_Line(x + r, y, x + length + size - r, y, size, color24);
	LCD_Line(x + r, y + width - 1, x + length + size - r, y + width - 1, size, color24);
	LCD_Line(x, y + r, x, y + width - size - r, size, color24);
	LCD_Line(x + length - 1, y + r, x + length - 1, y + width - size - r, size, color24);

	LCD_Circle_Helper(x + r, y + r, r, 1, size, color24);
	LCD_Circle_Helper(x + length - r - 1, y + r, r, 2, size, color24);
	LCD_Circle_Helper(x + length - r - 1, y + width - r - 1, r, 4, size, color24);
	LCD_Circle_Helper(x + r, y + width - r - 1, r, 8, size, color24);
}

void LCD_Circle_Fill_Helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint32_t color24)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x1) {
			LCD_Line(x0 + x, y0 - y, x0 + x, y0 - y + 2 * y + delta, 1, color24);
			LCD_Line(x0 + y, y0 - x, x0 + y, y0 - x + 2 * x + delta, 1, color24);
		}
		if (cornername & 0x2) {
			LCD_Line(x0 - x, y0 - y, x0 - x, y0 - y + 2 * y + delta, 1, color24);
			LCD_Line(x0 - y, y0 - x, x0 - y, y0 - x + 2 * x + delta, 1, color24);
		}
	}
}

void LCD_Rect_Round_Fill(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t r, uint32_t color24)
{
	fillRect(x + r, y, length - 2 * r, width, color24);
	LCD_Circle_Fill_Helper(x + length - r - 1, y + r, r, 1, width - 2 * r - 1, color24);
	LCD_Circle_Fill_Helper(x + r, y + r, r, 2, width - 2 * r - 1, color24);
}
