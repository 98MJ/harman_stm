#include "I2C_CLCD.h"

extern I2C_HandleTypeDef hi2c1; // 중복 선언 재정의 (hi2c1)

void I2C_CLCD_Delay_us(uint8_t us)
{
	volatile uint8_t i;

	for(i = 0; i < 19*us/10; i++);
}

void I2C_CLCD_SendByte(uint8_t RS_State, uint8_t Byte)
{
	uint8_t i;
	uint8_t buffer[4];

	for(i = 0; i < 2; i++)
	{
		buffer[i] = (Byte & 0xf0) | (1 << I2C_CLCD_LED) | (!i << I2C_CLCD_E) | (0 << I2C_CLCD_RW) | (RS_State << I2C_CLCD_RS);
	}

	for(i = 0; i < 2; i++)
	{
		buffer[i+2] = (Byte << 4) | (1 << I2C_CLCD_LED) | (!i << I2C_CLCD_E) | (0 << I2C_CLCD_RW) | (RS_State << I2C_CLCD_RS);
	}
	HAL_I2C_Master_Transmit(&hi2c1, PCF8574_AD | WRITE, buffer, 4, 300);

	I2C_CLCD_Delay_us(40);
}

void I2C_CLCD_init(void)
{
	uint8_t i;
	uint8_t CLCD_Init_CMD[4] = {0x28, 0x0c, 0x01, 0x06};

	HAL_Delay(100);

	I2C_CLCD_SendByte(0, 0x02);

	HAL_Delay(2);

	for(i = 0; i < 4; i++)
	{
		I2C_CLCD_SendByte(0, CLCD_Init_CMD[i]);

		if(i == 2) HAL_Delay(2);
	}
}

void I2C_CLCD_GotoXY(uint8_t X, uint8_t Y)
{
	I2C_CLCD_SendByte(0, 0x80 | (0x40 * Y + X));
}

void I2C_CLCD_PutC(uint8_t C)
{
	if(C == '\f')
	{
		I2C_CLCD_SendByte(0, 0x01);
		HAL_Delay(2);
	}
	else if(C == '\n')
	{
		I2C_CLCD_GotoXY(0, 1);
	}
	else
	{
		I2C_CLCD_SendByte(1, C);
	}
}

void I2C_CLCD_PutStr(uint8_t *Str)
{
	while(*Str) I2C_CLCD_PutC(*Str++);
}

void I2C_CLCD_Cursor(uint8_t on){
	I2C_CLCD_SendByte(0, 0x0c | (on<< 1));
}
