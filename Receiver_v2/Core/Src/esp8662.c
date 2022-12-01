/*
 * esp8662.c
 *
 *  Created on: 22 lis 2022
 *      Author: jasie
 */

#include "esp8662.h"

#define UART_TIMEOUT 1000

static uint8_t UART_Send(const char* data){
	if(HAL_UART_Transmit(huartdma.huart, (uint8_t*)data, strlen(data), UART_TIMEOUT) != HAL_OK){
		return 0;
	}
	return 1;
}

/*
 * TODO: comment
 */
static ESP_ERROR_CODE ESP_FindResponse(const char* resp, uint16_t wait){
	char line[256];
	uint8_t idx = 0;
	uint32_t real_wait = 2*wait;
	HAL_TIM_Base_Start(&htim3);
	while(__HAL_TIM_GET_COUNTER(&htim3) <= real_wait);
	TIM3->CNT = 0;
	HAL_TIM_Base_Stop(&htim3);
	while(UARTDMA_IsDataReady(&huartdma) && idx < 256){
		line[idx] = UARTDMA_GetCharFromBuffer(&huartdma);
		idx++;
	}
	if(strstr(line, resp) != NULL){
		return ESP_OK;
	}
}

/*
 * @brief initializes ESP module as end device with single connection
 * @params: None
 *
 * @return: one of the error codes: success, failure, timeout
 * @retval: 0,1,2
 */
ESP_ERROR_CODE ESP_Init(){
	if(!UART_Send("ATE0\r\n")){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("OK", 200) != ESP_OK){
		return ESP_ERROR;
	}
	// reset esp
	if(!UART_Send("AT+RST\r\n")){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("OK", 200) != ESP_OK){
		return ESP_ERROR;
	}

	// set mode to end device
	if(!UART_Send("AT+CWMODE=1\r\n")){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("OK", 200) != ESP_OK){
		return ESP_ERROR;
	}

	// set single connection
	if(!UART_Send("AT+CIPMUX=0\r\n")){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("OK", 200) != ESP_OK){
		return ESP_ERROR;
	}
	return ESP_OK;
}

/*
 * @brief connects to specified access point
 * @param[in] wifi_ssid : pointer to string with wifi ssid
 * @param[in] wifi_pass : pointer to string with wifi password
 *
 * @return: one of the error codes: success, failure, timeout
 * @retval: 0,1,2
 */
ESP_ERROR_CODE ESP_WiFiConnect(const char* wifi_ssid, const char* wifi_pass){
	static char wifi_access[100];
	sprintf(wifi_access, "AT+CWJAP=\"%s\",\"%s\"\r\n", wifi_ssid, wifi_pass);

	// make connection
	if(!UART_Send(wifi_access)){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("OK", 2000) != ESP_OK){
		return ESP_ERROR;
	}
	return ESP_OK;
}

/*
 * TODO: comment
 */
void ESP_WeatherDataPrepare(WeatherDataToSend* weather_data, RawMeasurements* raw_data){
	weather_data->base_float_humidity = raw_data->base_am2320_humidity / 10.;
	weather_data->base_float_temperature = (raw_data->base_am2320_temperature / 10. + raw_data->base_bmp280_temp / 100.) / 2.;
	weather_data->base_float_press = raw_data->base_bmp280_press / 25600.;
	weather_data->remote_float_humidity = 0.0;
	weather_data->remote_float_press = 0.0;
	weather_data->remote_float_temperature = 0.0;
}

/*
 * @brief sends very specifically formatted data to server with given ip
 * @param[in] ip_address : pointer to string with ipv4 address
 * @param[in] data : pointer to structure with accumulated weather data
 *
 * @return: one of the error codes: success, failure, timeout
 * @retval: 0,1,2
 */
ESP_ERROR_CODE ESP_SendData(const char* ip_address, WeatherDataToSend* data){
	// prepare string
	static char data_buffer[200];
	sprintf(data_buffer, "GET https://api.thingspeak.com/update?api_key=DCFXM8NU6FU9K22B&field1=%2.2f&field2=%2.2f&field3=%4.3f&field4=%4.3f&field5=%2.2f&field6=%2.2f\r\n",
			data->remote_float_temperature, data->base_float_temperature, data->remote_float_press,
			data->base_float_press, data->remote_float_humidity, data->base_float_humidity);

//	sprintf(data_buffer, "GET https://api.thingspeak.com/update?api_key=DCFXM8NU6FU9K22B&field1=11&field2=11&field3=11&field4=11&field5=11&field6=11\r\n");
	static char send_command[24];
	sprintf(send_command, "AT+CIPSEND=%d\r\n", strlen(data_buffer));

	// connect to thingspeak
	char webserver_buffer[25+strlen(ip_address)+2];
	sprintf(webserver_buffer, "AT+CIPSTART=\"TCP\",\"%s\",80\r\n", ip_address);
	if(!UART_Send(webserver_buffer)){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("OK", 500) != ESP_OK){
		return ESP_ERROR;
	}

	// start sending
	if(!UART_Send(send_command)){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("<", 500) != ESP_OK){
		return ESP_ERROR;
	}

	// send data
	if(!UART_Send(data_buffer)){
		return ESP_ERROR;
	}
	if(ESP_FindResponse("SEND OK", 2000) != ESP_OK){
		return ESP_ERROR;
	}
	return ESP_OK;

}
