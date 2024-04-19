#include "uart.h"

UART_HandleTypeDef *myHuart1;
UART_HandleTypeDef *myHuart2;

#define rxBufferMax		255

int rxBufferGp1;
int rxBufferPp1;
uint8_t rxBuffer1[rxBufferMax];
uint8_t rxChar1;

int rxBufferGp2;
int rxBufferPp2;
uint8_t rxBuffer2[rxBufferMax];
uint8_t rxChar2;

void initUART1(UART_HandleTypeDef *inHuart){
	myHuart1 = inHuart;
	HAL_UART_Receive_IT(myHuart1, &rxChar1, 1);
}
void initUART2(UART_HandleTypeDef *inHuart){
	myHuart2 = inHuart;
	HAL_UART_Receive_IT(myHuart2, &rxChar2, 1);
}
// 매개변수 : 구조체(UART_HandleTypeDef)
// inHuart를 myHuart에 할당
// myHuart 수신 인터럽트 활성화


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == myHuart1){
		rxBuffer1[rxBufferPp1++] = rxChar1;
		rxBufferPp1 %= rxBufferMax;
		HAL_UART_Receive_IT(myHuart1, &rxChar1, 1);
	}
	if(huart == myHuart2){
		rxBuffer2[rxBufferPp2++] = rxChar2;
		rxBufferPp2 %= rxBufferMax;
		HAL_UART_Receive_IT(myHuart2, &rxChar2, 1);
	}
}
// UART 수신 완료 콜백 함수
// UART 수신이 완료되면 HAL_UART_Receive_IT 호출
// rxChar : 수신된 데이터
// 수신된 데이터를 rxBuffer에 저장, Pp증가
// 원형 버퍼

uint8_t getChar1(){
	uint8_t result;
	if(rxBufferGp1 == rxBufferPp1) return 0;
	result = rxBuffer1[rxBufferGp1++];
	rxBufferGp1 %= rxBufferMax;
	return result;
}
uint8_t getChar2(){
	uint8_t result;
	if(rxBufferGp2 == rxBufferPp2) return 0;
	result = rxBuffer2[rxBufferGp2++];
	rxBufferGp2 %= rxBufferMax;
	return result;
}
// buffer에서 문자 하나를 가져오는 함수
// Gp = Pp -> 버퍼가 비어있다. -> return 0
// Gp != Pp -> rxBuffer[rxBufferGp] 값을 가져오고, Gp++


int _write(int file, char *p, int len){
	HAL_UART_Transmit(myHuart2, p, len, 10);
	return len;
}
