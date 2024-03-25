#include "I2C_CLCD.h"

extern I2C_HandleTypeDef hi2c1; // 중복 선언 재정의 (hi2c1)

void I2C_CLCD_Delay_us(uint8_t us) {
	volatile uint8_t i;

	for (i = 0; i < 19 * us / 10; i++)
		;
}

void I2C_CLCD_SendByte(uint8_t RS_State, uint8_t Byte) {
	uint8_t i;
	uint8_t buffer[4]; // pcf8574의 데이터 전송 핀 수 = 4

	for (i = 0; i < 2; i++) { // buffer[0], buffer[1]
		buffer[i] = (Byte & 0xf0) | (1 << I2C_CLCD_LED) | (!i << I2C_CLCD_E)
				| (0 << I2C_CLCD_RW) | (RS_State << I2C_CLCD_RS);
	}

	for (i = 0; i < 2; i++) { // buffer[2], buffer[3]
		buffer[i + 2] = (Byte << 4) | (1 << I2C_CLCD_LED) | (!i << I2C_CLCD_E)
				| (0 << I2C_CLCD_RW) | (RS_State << I2C_CLCD_RS);
	}
	HAL_I2C_Master_Transmit(&hi2c1, PCF8574_AD | WRITE, buffer, 4, 300);
	// (i2c1, slave_address+W, data, size, timeout)
	// RS_Stata = 0:cmd, 1:data

	I2C_CLCD_Delay_us(40);
}

void I2C_CLCD_init(void) {
	uint8_t i;
	uint8_t CLCD_Init_CMD[4] = { 0x28, 0x0c, 0x01, 0x06 };

	HAL_Delay(100);

	I2C_CLCD_SendByte(0, 0x02);

	HAL_Delay(2);

	for (i = 0; i < 4; i++) {
		I2C_CLCD_SendByte(0, CLCD_Init_CMD[i]);

		if (i == 2)
			HAL_Delay(2);
	}
}

void I2C_CLCD_GotoXY(uint8_t X, uint8_t Y) {
	I2C_CLCD_SendByte(0, 0x80 | (0x40 * Y + X));
}

void I2C_CLCD_PutC(uint8_t C) {
	if (C == '\f') {
		I2C_CLCD_SendByte(0, 0x01);
		HAL_Delay(2);
	} else if (C == '\n') {
		I2C_CLCD_GotoXY(0, 1);
	} else {
		I2C_CLCD_SendByte(1, C);
	}
}

void I2C_CLCD_PutStr(uint8_t *Str) {
	while (*Str)
		I2C_CLCD_PutC(*Str++);
}

void I2C_CLCD_Cursor(uint8_t on) {
	I2C_CLCD_SendByte(0, 0x0c | (on << 1));
}

void I2C_CLCD_Right() {
	I2C_CLCD_SendByte(0, 0b00011100); // 뒤에 두 비트는 don't care
}
void I2C_CLCD_Left() {
	I2C_CLCD_SendByte(0, 0b00011000);
}
void I2C_CLCD_ShiftStop() {
	I2C_CLCD_SendByte(0, 0b00000100);
}

void I2C_CLCD_CustomFont() {
	I2C_CLCD_SendByte(0, 0x40); //0b0100 0000
	/* I2C_CLCD_SendByte(0, 0x48); //0b0100 1000
	 I2C_CLCD_SendByte(0, 0x50); //0b0101 0000 */ // 주소 자동 증가
	for (int i = 0; i < 8; i++) {
		I2C_CLCD_SendByte(1, 0b00000);
	}
	for (int i = 0; i < 8; i++) {
		I2C_CLCD_SendByte(1, 0b10000);
	}
	for (int i = 0; i < 8; i++) {
		I2C_CLCD_SendByte(1, 0b11000);
	}
	for (int i = 0; i < 8; i++) {
		I2C_CLCD_SendByte(1, 0b11100);
	}
	for (int i = 0; i < 8; i++) {
		I2C_CLCD_SendByte(1, 0b11110);
	}
	for (int i = 0; i < 8; i++) {
		I2C_CLCD_SendByte(1, 0b11111);
	}
}

void I2C_CLCD_Progressbar(uint8_t n, uint8_t line){
	I2C_CLCD_GotoXY(0, line);
	for(int i=0; i<(n/5); i++){
		I2C_CLCD_PutC(5); // 5*8이 풀로 채워진 것
	}
	I2C_CLCD_PutC(n % 5);
	// ex) n이 26이면, 5*5 + 1

	for(int i=(n/5)+1; i<16; i++){
		I2C_CLCD_PutC(0); // 빈칸은 0으로
	}
}
