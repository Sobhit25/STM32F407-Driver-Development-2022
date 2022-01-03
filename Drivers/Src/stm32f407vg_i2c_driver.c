/*
 * stm32f407vg_i2c.c
 *
 *  Created on: 13-Sep-2021
 *      Author: sobhit25
 */


#include <stm32f407vg_i2c_driver.h>

static uint16_t HPRE[] = {2,4,8,16,32,64,128,256,512};
static uint8_t PPRE1[]= {2,4,8,16};

 uint32_t Get_RCC_PLL_Clk(void){
	 uint32_t temp = 0;

	 return temp;
 }

uint32_t RCC_GetPClk1Value(void){

	uint32_t pClk1;

	// Find the Clock Source
	uint8_t clksrc = (RCC->CFGR >> 2) & 0x3;
	uint32_t SystemClk;
	if(clksrc == 0)
		SystemClk = 16000000;
	else if(clksrc == 1)
		SystemClk = 8000000;
	else if(clksrc == 2)
		SystemClk = Get_RCC_PLL_Clk();

	//Find the AHB Pre-Scaler
	uint8_t temp1 = (RCC->CFGR >> 4) & 0xFF;
	uint8_t AHBPrescaler;
	if(temp1<8)
		AHBPrescaler = 1;
	else
		AHBPrescaler = HPRE[temp1-8];

	//Find the APB1 Pre-Scaler
	uint8_t temp2 = (RCC->CFGR >> 10) & 0x7;
	uint8_t APB1Prescaler;
	if(temp2<4)
		APB1Prescaler = 1;
	else
		APB1Prescaler = PPRE1[temp2-8];

	//Calculate Peripheral CLock
	pClk1 = (SystemClk/AHBPrescaler)/APB1Prescaler;
	return pClk1;
}



/********************************************************************************
 * 	**							I2C DRIVER APIs								**	*
 ********************************************************************************/


/*
 * SPI Peripheral clock control function
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1)
			I2C1_PClk_EN();
		else if(pI2Cx == I2C2)
			I2C2_PClk_EN();
		else if(pI2Cx == I2C3)
			I2C3_PClk_EN();
	}
	else{
		if(pI2Cx == I2C1)
			I2C1_PClk_DI();
		else if(pI2Cx == I2C2)
			I2C2_PClk_DI();
		else if(pI2Cx == I2C3)
			I2C3_PClk_DI();
	}
}

/*
 * I2C Init De-Init functions
 */
void I2C_Init(I2C_Handle_t* pI2CHandle){
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempreg = 0;

	//1. Enable or Disable Acking in CR1 Register
	tempreg |= (pI2CHandle->I2C_Config.AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	tempreg = 0;

	//2. Configure the Frequency of CR2 Register
	tempreg |= (RCC_GetPClk1Value()/1000000);
	pI2CHandle->pI2Cx->CR2 =tempreg;

	tempreg = 0;

	//3. Program the Device with its Own Address
	tempreg |= (pI2CHandle->I2C_Config.DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 =tempreg;

	//4. CCR Calculations
	tempreg = 0;
	uint32_t ccr;

	//Standard Mode
	if(pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCLSpeed_SM_100){
		ccr = RCC_GetPClk1Value()/(2*pI2CHandle->I2C_Config.SCLSpeed);
		tempreg |= (ccr & 0xFFF);
	}
	//Fast Mode
	else{
		tempreg |= (1 << I2C_CCR_FS);
		//Duty - 0
		if(pI2CHandle->I2C_Config.FMDutyCycle == I2C_FMDUTY_2){
			ccr = RCC_GetPClk1Value()/(3*pI2CHandle->I2C_Config.SCLSpeed);
			tempreg |= (ccr & 0xFFF);
		}
		//Duty - 1
		else{
			ccr = RCC_GetPClk1Value()/(25*pI2CHandle->I2C_Config.SCLSpeed);
			tempreg |= (ccr & 0xFFF);
			tempreg |= (1 << I2C_CCR_DUTY);
		}
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//5. TRISE Calculations
	tempreg=0;
		if(pI2CHandle->I2C_Config.SCLSpeed <= I2C_SCLSpeed_SM_100)
		{
			//mode is standard mode

			tempreg = (RCC_GetPClk1Value() /1000000U) + 1 ;

		}else
		{
			//mode is fast mode
			tempreg = ( (RCC_GetPClk1Value() * 300) / 1000000000U ) + 1;

		}

		pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1)
		I2C1_RESET();
	else if(pI2Cx == I2C2)
		I2C2_RESET();
	else if(pI2Cx == I2C3)
		I2C3_RESET();
}


/*
 * I2C Send & Recieve Data functions
 */

void I2CMaster_SendData(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t RepeatedSt){

	//Start Generation
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	//Assure the Completion of Start
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//Send the Slave Address with r/w' bit w(0)
	slaveAddr = ( slaveAddr << 1 ) ;
	pI2CHandle->pI2Cx->DR = slaveAddr;

	//Confirm the Address was sent by checking the ADDR Flag
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));
	//CLear the ADDR Flag
	uint32_t tempreg;
	tempreg = pI2CHandle->pI2Cx->SR1;
	tempreg = pI2CHandle->pI2Cx->SR2;
	(void)tempreg;

	while(len > 0){
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//Wait for TXE and BTF
	while(!((pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) & (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF))));
	//Generate Stop
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
void I2CMaster_RecieveData(I2C_Handle_t *pI2CHandle,uint8_t* pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t RepeatedSt){

	//Start Generation
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	//Assure the Completion of Start
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));

	//Send the Slave Address with r/w' bit r(1)
	slaveAddr = ( slaveAddr << 1 ) + 1;
	pI2CHandle->pI2Cx->DR = slaveAddr;

	//Confirm the Address was sent by checking the ADDR Flag
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));



	//procedure to read only 1 byte from slave
	if(len == 1)
	{
		//Disable Acking
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		//CLear the ADDR Flag
		uint32_t tempreg;
		tempreg = pI2CHandle->pI2Cx->SR1;
		tempreg = pI2CHandle->pI2Cx->SR2;
		(void)tempreg;

		//wait until  RXNE becomes 1
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));

		//generate STOP condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(len > 1)
	{
		//clear the ADDR flag
		uint32_t tempreg;
		tempreg = pI2CHandle->pI2Cx->SR1;
		tempreg = pI2CHandle->pI2Cx->SR2;
		(void)tempreg;

		//read the data until Len becomes zero
		for ( uint32_t i =len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				//generate STOP condition
				pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);


}



/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr, uint8_t RepeatedSt)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RepeatedSt = RepeatedSt;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t RepeatedSt)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RepeatedSt = RepeatedSt;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}




/*
 * I2C IRQ Configuration and Handling functions
 */

void I2C_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi){
	//Set-reset the interrupt
	if(EnorDi == ENABLE){
		if(IRQNumber >=0  &&  IRQNumber<32)
			*NVIC_ISER0 |= (1 << IRQNumber);
		else if(IRQNumber >=32  &&  IRQNumber<64)
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		else if(IRQNumber >=64  &&  IRQNumber<96)
			*NVIC_ISER2 |= (1 << IRQNumber%32);
	}
	else{
		if(IRQNumber >=0  &&  IRQNumber<32)
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		else if(IRQNumber >=32  &&  IRQNumber<64)
			*NVIC_ICER1 &= ~(1 << IRQNumber%32);
		else if(IRQNumber >=64  &&  IRQNumber<96)
			*NVIC_ICER2 &= ~(1 << IRQNumber%32);
	}

	//Configure the Priority
	uint8_t irp_reg = IRQNumber/4;
	uint8_t irp_pos = IRQNumber%4;
	uint8_t shift   = (irp_pos * 8) + (8 - NO_IPR_BITS);

	*(NVIC_IPR_BASE + irp_reg) |= (IRQPriority << shift);
}



/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Interrupt handling for different I2C events (refer SR1)

 */


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device
	//uint32_t temp1 = pI2CHandle->pI2Cx->CR2 & (1 << );
	//uint32_t temp2 = pI2CHandle->pI2Cx->CR2 & (1 << );

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set

	//5. Handle For interrupt generated by TXE event

	//6. Handle For interrupt generated by RXNE event

}



/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Interrupt handling for different I2C events (refer SR1)

 */


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

}
