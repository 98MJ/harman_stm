#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"

#define STX		0x02 //ASCII 코드
#define ETX		0x03 //ASCII 코드

typedef struct{
	uint8_t		id;
	uint8_t		command;
	uint8_t		data;
} protocol_t;


void initUART(UART_HandleTypeDef *inHuart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t getChar();
void binaryTransmit(protocol_t inData);

#endif /* INC_UART_H_ */
