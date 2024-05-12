/*
 * ugvMain.c
 *
 *  Created on: May 3, 2024
 *      Author: Mehmet Dincer
 */


#include "ugvMain.h"


//<<<<<<<<<<<<<<<-STATIC FUNCTIONS->>>>>>>>>>>>>>>>
static void systemInitialize();
static int loraInit(SX1278_hw_t *sx1278Hw, SX1278_t *sx1278);


//<<<<<<<<<<<<<<<<<<-GLOBAL VARIABLES->>>>>>>>>>>>>>>>>>>
static uint8_t loreRxTxMutex;
static int ret;
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
uint8_t loraRx[LORA_RX_SIZE];
uint8_t loraTx[LORA_TX_SIZE];
LoraData loraDataRx;
LoraData loraDataTx;
uint8_t loraSendFlag = 0;


/**
 * @brief system main function
 * @return none
 */
void ugvMain(void)
{
	systemInitialize();

	while(1)
	{
//		loreRxTxMutex = LORA_TX_STATUS;
//		ret = SX1278_LoRaEntryTx(&SX1278, 8, 2000);
//		ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) txMsg,
//								  8, LORA_TIMEOUT);

		loreRxTxMutex = LORA_RX_STATUS;
		ret = SX1278_LoRaEntryRx(&SX1278, sizeof(loraRx), LORA_TIMEOUT);

//		if(loraSendFlag == 1)
//		{
//			transtmitPackage(&loraDataRx);
//			loraSendFlag = 0;
//		}

		HAL_Delay(1000);
	}
}

/**
 * @brief initialize peripheral and drivers
 * @return none
 */
static void systemInitialize()
{
	init_PEC15_Table();
	loraInit(&SX1278_hw, &SX1278);
	HAL_IWDG_Init(&hiwdg);
}

/**
 * @brief set the pin configuration and initialize the lora
 * @param[in] lora hardware global variable
 * @param[in] lora global variable
 * @return 1 if success else 0(timeout)
 */
static int loraInit(SX1278_hw_t *sx1278Hw, SX1278_t *sx1278)
{
	int ret;

	sx1278Hw->dio0.port = DIO0_GPIO_Port;
	sx1278Hw->dio0.pin = DIO0_Pin;
	sx1278Hw->nss.port = NSS_GPIO_Port;
	sx1278Hw->nss.pin = NSS_Pin;
	sx1278Hw->reset.port = GPIOB;
	sx1278Hw->reset.pin = GPIO_PIN_1;
	sx1278Hw->spi = &hspi1;

	sx1278->hw = sx1278Hw;

	SX1278_init(sx1278, 434000000, SX1278_POWER_11DBM, SX1278_LORA_SF_7,
	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);

	ret = SX1278_LoRaTxPacket(sx1278, loraTx,
			8, LORA_TIMEOUT);

	return ret;
}

/**
 * @brief lora transmit and receive callback
 * @return none
 */
void loraDioCallBack()
{
	if(loreRxTxMutex == LORA_TX_STATUS)
	{
		//		ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) txMsg,
		//								  8, LORA_TIMEOUT);
	}
	else
	{
		ret = SX1278_available(&SX1278);

		SX1278_read(&SX1278, (uint8_t*)&loraDataRx, ret);

		uint16_t crc = AE_pec15((uint8_t*)&loraDataRx, (sizeof(LoraData) - 4));
		if(crc == (loraDataRx.crcLsb << 0 | loraDataRx.crcMsb << 8))
		{
			loraDataRx.carriage = '\r';
			loraDataRx.newline = '\n';

			transtmitPackage(&loraDataRx);
		}

		HAL_IWDG_Refresh(&hiwdg);
	}
}

/**
 * @brief update the values and trasmit to pc
 */

void transtmitPackage(LoraData * loraDat)
{
	uint8_t dataBuffer[52];
	uint32_t u32TempVar;

	dataBuffer[0] = (loraDat->azimuth >> 0) & 0xFF;			//!< azimuth lsb
	dataBuffer[1] = (loraDat->azimuth >> 8) & 0xFF;			//!< azimuth msb
	dataBuffer[2] = loraDat->latitudeDegree;				//!< latitude degree
	dataBuffer[3] = loraDat->latitudeMinute;				//!< latitude minute

	u32TempVar = *((uint32_t*)&loraDat->latitudeSecond);	//!< latitude second
	dataBuffer[4] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[5] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[6] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[7] = (u32TempVar >> 24) & 0xFF;

	dataBuffer[8] = loraDat->longitudeDegree;				//!< longitude degree
	dataBuffer[9] = loraDat->longitudeMinute;				//!< longitude minute
	dataBuffer[10] = loraDat->numberOfSatellite;			//!< number of satellite
	dataBuffer[11] = loraDat->second;						//!< gps second

	u32TempVar = *((uint32_t*)&loraDat->longitudeSecond);	//!< longitude second
	dataBuffer[12] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[13] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[14] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[15] = (u32TempVar >> 24) & 0xFF;

	u32TempVar = *((uint32_t*)&loraDat->speed);				//!< gps speed
	dataBuffer[16] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[17] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[18] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[19] = (u32TempVar >> 24) & 0xFF;

	u32TempVar = *((uint32_t*)&loraDat->Ax);				//!< mpu accelartion x
	dataBuffer[20] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[21] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[22] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[23] = (u32TempVar >> 24) & 0xFF;

	u32TempVar = *((uint32_t*)&loraDat->Ay);				//!< MPU acceleration y
	dataBuffer[24] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[25] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[26] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[27] = (u32TempVar >> 24) & 0xFF;

	u32TempVar = *((uint32_t*)&loraDat->Temperature);		//!< MPU temperature
	dataBuffer[28] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[29] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[30] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[31] = (u32TempVar >> 24) & 0xFF;

	u32TempVar = *((uint32_t*)&loraDat->KalmanAngleX);		//!< MPU kalman x angle
	dataBuffer[32] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[33] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[34] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[35] = (u32TempVar >> 24) & 0xFF;

	u32TempVar = *((uint32_t*)&loraDat->KalmanAngleY);		//!< MPU kalman y angle
	dataBuffer[36] = (u32TempVar >> 0) & 0xFF;
	dataBuffer[37] = (u32TempVar >> 8) & 0xFF;
	dataBuffer[38] = (u32TempVar >> 16) & 0xFF;
	dataBuffer[39] = (u32TempVar >> 24) & 0xFF;

	dataBuffer[40] = loraDat->minute;						//!< gps minute
	dataBuffer[41] = loraDat->hour;							//!< gps hour
	dataBuffer[42] = loraDat->day;							//!< gps day
	dataBuffer[43] = loraDat->month;						//!< gps month

	dataBuffer[44] = loraDat->locationLat;					//!< gps latitude location
	dataBuffer[45] = loraDat->locationLong;					//!< gps longitude location
	dataBuffer[46] = loraDat->ledState;
	dataBuffer[47] = loraDat->gpsState;

	uint16_t pec = AE_pec15((uint8_t*)dataBuffer, 48);
	dataBuffer[48] = (pec >> 0) & 0xFF;
	dataBuffer[49] = (pec >> 8) & 0xFF;
	dataBuffer[50] = loraDat->carriage;
	dataBuffer[51] = loraDat->newline;

	HAL_UART_Transmit(&huart2, dataBuffer, sizeof(dataBuffer), 52);
}














