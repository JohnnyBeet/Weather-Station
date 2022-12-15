/*
 * esp8662.h
 *
 *  Created on: 22 lis 2022
 *      Author: jasie
 */

#ifndef INC_ESP8662_H_
#define INC_ESP8662_H_

#include "main.h"
#include <string.h>
#include "uart_dma.h"
#include <stdio.h>

#define MEAS_DATA_LEN 36

extern UARTDMA_HandleTypeDef huartdma;
extern TIM_HandleTypeDef htim3;


typedef struct{
	int32_t remote_bmp280_temp;
	uint32_t remote_bmp280_press;
	int16_t remote_am2320_temperature;
	uint16_t remote_am2320_humidity;

	int32_t base_bmp280_temp;
	uint32_t base_bmp280_press;
	int16_t base_am2320_temperature;
	uint16_t base_am2320_humidity;
}RawMeasurements;

typedef struct{
	float remote_float_press;
	float remote_float_temperature;
	float remote_float_humidity;

	float base_float_press;
	float base_float_temperature;
	float base_float_humidity;
}WeatherDataToSend;

typedef enum{
	ESP_OK = (uint8_t)0,
	ESP_ERROR = (uint8_t)1,
	ESP_TIMEOUT = (uint8_t)2
}ESP_ERROR_CODE;

ESP_ERROR_CODE ESP_Init(void);
ESP_ERROR_CODE ESP_WiFiConnect(const char* wifi_ssid, const char* wifi_pass);
ESP_ERROR_CODE ESP_SendData(const char* ip_address, WeatherDataToSend* data, int baseFlag);
void ESP_WeatherDataPrepare(WeatherDataToSend* weather_data, RawMeasurements* raw_data, int baseFlag);

#endif /* INC_ESP8662_H_ */
