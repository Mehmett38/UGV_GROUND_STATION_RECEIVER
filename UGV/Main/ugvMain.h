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
#include "iwdg.h"

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

typedef enum{
	NORTH,
	SOUTH,
	EAST,
	WEST,
}Location;

//<<<<<<<<<<<<<<<<<<<<<-STRUCTURES->>>>>>>>>>>>>>>>>>>

typedef struct{
	uint16_t azimuth;

	uint8_t latitudeDegree;	//!< this variables are float convert them HEX form
	uint8_t latitudeMinute;	//!< this variables are float convert them HEX form
	float latitudeSecond;	//!< this variables are float convert them HEX form

	uint8_t longitudeDegree;	//!< this variables are float convert them HEX form
	uint8_t longitudeMinute;	//!< this variables are float convert them HEX form
	uint8_t numberOfSatellite;
	uint8_t second;
	float longitudeSecond;	//!< this variables are float convert them HEX form


	float speed;				//!< this variables are float convert them HEX form
	float Ax;
	float Ay;

    float Temperature;

    float KalmanAngleX;
    float KalmanAngleY;

	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;

    Location locationLat;
    Location locationLong;

	LED_STATE ledState;

	GPS_State gpsState;

	uint8_t crcLsb;
	uint8_t crcMsb;

	uint8_t carriage;		//!< \r
	uint8_t newline;		//!< \n

}LoraData;

//<<<<<<<<<<<<<<<-FUNCTION PROTOTYPES->>>>>>>>>>>>>>>>
void ugvMain(void);
void loraDioCallBack();
void transtmitPackage(LoraData * loraDat);


#endif /* MAIN_UGVMAIN_H_ */
