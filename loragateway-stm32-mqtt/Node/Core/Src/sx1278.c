/*
 * sx1278.c
 *
 *  Created on: Nov 20, 2024
 *      Author: khoav
 */
#include "main.h"
#include "sx1278.h"

uint8_t data_spi_tx[2]={};
uint8_t data_spi_rx[2]={};

uint8_t readRegister(uint8_t addr) {
	CS = 0;
	data_spi_tx[0]= addr & 0x7F;
	HAL_SPI_Transmit(&hspi1, data_spi_tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data_spi_rx, 1, HAL_MAX_DELAY);
	CS = 1;
	return data_spi_rx[0];
}

void writeRegister(uint8_t addr, uint8_t data) {
	CS = 0;
	data_spi_tx[0]= addr | 0x80;
	HAL_SPI_Transmit(&hspi1, data_spi_tx, 1, HAL_MAX_DELAY);
	data_spi_tx[0]= data;
	HAL_SPI_Transmit(&hspi1, data_spi_tx, 1, HAL_MAX_DELAY);
	CS = 1;
}

void standby_mode() {
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sleep_mode() {
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void tx_mode() {
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void rx_mode() {
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void setFrequency(unsigned long frequency) {
	writeRegister(REG_FRF_MSB, (uint8_t) (frequency >> 16));
	writeRegister(REG_FRF_MID, (uint8_t) (frequency >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t) (frequency >> 0));
}

void sx1278_init(unsigned long frequency) {
	RST_LORA = 0;
	HAL_Delay(1);
	RST_LORA = 1;
	HAL_Delay(1);
	standby_mode();
	HAL_Delay(1);
	sleep_mode();

	setFrequency(frequency);

	writeRegister(0x1d, 0x72); //BW = 125khz, CR = 4/5
	writeRegister(0x1e, 0x70); //SF = 7

	writeRegister(0x0e, 0);
	writeRegister(0x0f, 0);
	writeRegister(0x0c, 0x23);
	writeRegister(0x26, 0x04);
	writeRegister(0x4d, 0x84);
	writeRegister(0x0b, 0x2b);
	writeRegister(0x09, 0x8f);
	standby_mode();
}


