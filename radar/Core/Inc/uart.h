#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"

#define STX		0x02 //ASCII 코드
#define ETX		0x03 //ASCII 코드

typedef struct{
	uint8_t		command;
	uint16_t	data;
} protocol_t;

void initUART(UART_HandleTypeDef *inHuart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int16_t getChar();
void transmitPacket(protocol_t data);
protocol_t receivePacket();

#endif /* INC_UART_H_ */
