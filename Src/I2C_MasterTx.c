/*
 * I2C_MasterTx.c
 *
 *  Created on: 18-Sep-2021
 *      Author: sobhit25
 */

#include <stm32f407vg.h>
#include <string.h>

void delay(void){
	for(uint32_t i=0;i<50000;i++){

	}
}

int main(){

	//GPIO Button
	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinNumber =	0;
	Button.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NOPUPD;

	GPIO_Init(&Button);
	GPIO_IRQConfig(IRQ_NO_EXTI0, 15, ENABLE);

	/*
	 * 		@Mapping:
	 * 		PB6 --> SCL
	 * 		PB7 --> SDA
	 */

	/*
	 * GPIO INIT FOR I2C PERIPH
	 */
	GPIO_Handle_t I2C_Pins;

	I2C_Pins.pGPIOx = GPIOB;

	I2C_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_OD;
	I2C_Pins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_PU;
	I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
	I2C_Pins.GPIO_PinConfig.GPIO_PinAltFunc = GPIO_AF4;

	//SCL Pin
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&I2C_Pins);

	//SDA Pin
	I2C_Pins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&I2C_Pins);



	I2C_Handle_t 	I2C1_Handler;

	I2C1_Handler.pI2Cx = I2C1;

	I2C1_Handler.I2C_Config.AckControl = I2C_ACK_ENABLE;
	I2C1_Handler.I2C_Config.DeviceAddress = 0b1010111;
	I2C1_Handler.I2C_Config.SCLSpeed = I2C_SCLSpeed_SM_100;


	I2C_Init(&I2C1_Handler);



	while(1);

}

void EXTI0_IRQHandler(void){
	delay();
	char data[] = {0x6B, 0x00};
	char data1[] = {0x1B, 0x00};

	(void) data;
	(void) data1;

	GPIO_IRQHandling(0);

	I2C1->CR1 |= (1 << I2C_CR1_PE);
	I2CMaster_SendData(I2C1, (uint8_t*) data, strlen(data), 0b1111111,0);
	delay();
	I2CMaster_SendData(I2C1, (uint8_t*) data1,strlen(data1),0b110100,0);

}
