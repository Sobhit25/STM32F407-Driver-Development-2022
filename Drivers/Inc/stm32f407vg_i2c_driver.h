/*
 * stm32f407vg_i2c.h
 *
 *  Created on: 13-Sep-2021
 *      Author: sobhit25
 */

#ifndef INC_STM32F407VG_I2C_DRIVER_H_
#define INC_STM32F407VG_I2C_DRIVER_H_

#include<stm32f407vg.h>

/*
 * I2C Configuration Stucture
 */
typedef struct{
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t AckControl;
	uint8_t FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

	uint8_t* pTxBuffer;
	uint32_t TxLen;

	uint8_t* pRxBuffer;
	uint32_t RxLen;
	uint32_t RxSize;

	uint8_t TxRxState;
	uint8_t DevAddr;
	uint8_t RepeatedSt;
}I2C_Handle_t;



/**********************************************************************************
 * 							 User Configurable Macros
 **********************************************************************************/

/*
 * @SCLSpeed
 */
#define I2C_SCLSpeed_SM_100		100000U
#define I2C_SCLSpeed_FM_400		400000U
#define I2C_SCLSpeed_FM_200		200000U

/*
 * @AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @FMDutyCycle
 */
#define I2C_FMDUTY_2		0
#define I2C_FMDUTY_16_9		1

/*
 * @TxRxState
 */
#define I2C_BUSY_IN_TX 		0
#define I2C_BUSY_IN_RX		1

/********************************************************************************
 * 	**							I2C DRIVER APIs								**	*
 ********************************************************************************/


/*
 * I2CI Peripheral clock control function
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);


/*
 * I2C Init De-Init functions
 */
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * I2C Send & Recieve Data functions
 */
void I2CMaster_SendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t RepeatedSt);
void I2CMaster_RecieveData(I2C_Handle_t* pI2CHandle,uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t RepeatedSt);

uint8_t I2C_SendDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t RepeatedSt);
uint8_t I2C_RecieveDataIT(I2C_Handle_t* pI2CHandle,uint8_t* pRxBuffer, uint32_t len, uint8_t RepeatedSt);


/*
 * I2C IRQ Configuration and Handling functions
 */

void I2C_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);





#endif /* INC_STM32F407VG_I2C_DRIVER_H_ */
