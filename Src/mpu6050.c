/*
 * mpu6050.c
 *
 *  Created on: 29-Oct-2021
 *      Author: sobhit25
 */


#include<stm32f407vg.h>
#include<stdio.h>

void sleep(){
	for(int i=0;i<100000;i++){

	}
}





// bus address 0x68
static int addr = 0x68;


static void mpu6050_reset(I2C_Handle_t *i2c_handle) {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    I2CMaster_SendData(i2c_handle, buf, 2, addr, 0);
    buf[0] = 0x1C;
    buf[1] = 0x10;
    I2CMaster_SendData(i2c_handle, buf, 2, addr, 0);
}


static void mpu6050_read_raw(I2C_Handle_t *i2c_handle, int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    I2CMaster_SendData(i2c_handle, &val, 1, addr, 1); // true to keep master control of bus
    I2CMaster_RecieveData(i2c_handle, buffer, 6, addr, 0);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    I2CMaster_SendData(i2c_handle, &val, 1, addr, 1); // true to keep master control of bus
    I2CMaster_RecieveData(i2c_handle, buffer, 6, addr, 0);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    I2CMaster_SendData(i2c_handle, &val, 1, addr, 1); // true to keep master control of bus
    I2CMaster_RecieveData(i2c_handle, buffer, 2, addr, 0); // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}


int main() {
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



    mpu6050_reset(&I2C1_Handler);

    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        mpu6050_read_raw(&I2C1_Handler, acceleration, gyro, &temp);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf(" %f %f %f\n", (float) (9.8*acceleration[0]/4096), (float) (9.8*acceleration[1]/4096), (float)(9.8*acceleration[2]/4096));

        ////printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        ////printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        sleep();
    }


    return 0;
}
