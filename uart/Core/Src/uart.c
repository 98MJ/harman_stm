#include "uart.h"

UART_HandleTypeDef *myHuart;

#define rxBufferMax		255

int rxBufferGp;
int rxBufferPp;
uint8_t rxBuffer[rxBufferMax];
uint8_t rxChar;

void initUART(UART_HandleTypeDef *inHuart){
	myHuart = inHuart;
	HAL_UART_Receive_IT(myHuart, &rxChar, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	rxBuffer[rxBufferPp++] = rxChar;
	rxBufferPp %= rxBufferMax;
	HAL_UART_Receive_IT(myHuart, &rxChar, 1);
}

uint8_t getChar(){
	uint8_t result;
	if(rxBufferGp == rxBufferPp) return 0;
	result = rxBuffer[rxBufferGp++];
	rxBufferGp %= rxBufferMax;
	return result;
}

int _write(int file, char *p, int len){
	HAL_UART_Transmit(myHuart, p, len, 10);
	return len;
}
