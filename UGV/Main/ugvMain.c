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

		HAL_Delay(100);
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

		uint16_t crc = AE_pec15((uint8_t*)&loraDataRx, (sizeof(LoraData) - 6));
		if(crc == (loraDataRx.crcLsb << 0 | loraDataRx.crcMsb << 8))
		{
			loraDataRx.carriage = '\r';
			loraDataRx.newline = '\n';
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&loraDataRx, sizeof(LoraData));
		}
	}
}














