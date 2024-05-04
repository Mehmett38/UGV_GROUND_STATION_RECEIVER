/*
 * ugvMain.h
 *
 *  Created on: May 3, 2024
 *      Author: Mehmet Dincer
 */

#ifndef MAIN_UGVMAIN_H_
#define MAIN_UGVMAIN_H_

//<<<<<<<<<<<<<<<<<<<<<-LIBRARIES->>>>>>>>>>>>>>>>>>>
#include "stm32f3xx_hal.h"
#include "SX1278.h"
#include "SX1278_hw.h"
#include "crc15.h"
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"

//<<<<<<<<<<<<<<<<<<<<<-MACROS->>>>>>>>>>>>>>>>>>>
#define LORA_TIMEOUT			(3)
#define LORA_TX_SIZE			(28)
#define LORA_RX_SIZE			(28)
#define LORA_TX_STATUS			(0)
#define LORA_RX_STATUS			(1)

//<<<<<<<<<<<<<<<<<<<<<-ENUMS->>>>>>>>>>>>>>>>>>>
typedef enum{
	NO_CONNECTION,
	WRONG_DATA,
	POSITION_FIXED,
}GPS_State;

typedef enum{
	LEDS_OFF,
	FRONT_LED_ON,
	ALL_LED_ON,
}LED_STATE;

//<<<<<<<<<<<<<<<<<<<<<-STRUCTURES->>>>>>>>>>>>>>>>>>>

typedef struct{
	uint16_t azimuth;

	uint8_t latitudeDegree;	//!< this variables are float convert them HEX form
	uint8_t latitudeMinute;	//!< this variables are float convert them HEX form
	float latitudeSecond;	//!< this variables are float convert them HEX form

	uint8_t longitudeDegree;	//!< this variables are float convert them HEX form
	uint8_t longitudeMinute;	//!< this variables are float convert them HEX form
	float longitudeSecond;	//!< this variables are float convert them HEX form

	uint8_t numberOfSatellite;

	float speed;				//!< this variables are float convert them HEX form

	LED_STATE ledState;

	GPS_State gpsState;

	uint8_t crcLsb;
	uint8_t crcMsb;

	uint8_t carriage;		//!< \r
	uint8_t newline;		//!< \n

//	uint8_t azimuthLsb;
//	uint8_t azimuthMsb;
//
//	uint8_t latitudeDegree;
//	uint8_t latitudeMinute;
//	uint8_t latitudeSecond;

//	uint8_t longitudeDegree;
//	uint8_t longitudeMinute;
//	uint8_t longitudeSecond;

//	uint8_t numberOfSatellite;

//	uint8_t height;
//	uint32_t speed;
}LoraData;

//<<<<<<<<<<<<<<<-FUNCTION PROTOTYPES->>>>>>>>>>>>>>>>
void ugvMain(void);
void loraDioCallBack();


#endif /* MAIN_UGVMAIN_H_ */
