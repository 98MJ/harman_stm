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
// 매개변수 : 구조체(UART_HandleTypeDef)
// inHuart를 myHuart에 할당
// myHuart 수신 인터럽트 활성화


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	rxBuffer[rxBufferPp++] = rxChar;
	rxBufferPp %= rxBufferMax;
	HAL_UART_Receive_IT(myHuart, &rxChar, 1);
}
// UART 수신 완료 콜백 함수
// UART 수신이 완료되면 HAL_UART_Receive_IT 호출
// rxChar : 수신된 데이터
// 수신된 데이터를 rxBuffer에 저장, Pp증가
// 원형 버퍼


uint8_t getChar(){
	uint8_t result;
	if(rxBufferGp == rxBufferPp) return 0;
	result = rxBuffer[rxBufferGp++];
	rxBufferGp %= rxBufferMax;
	return result;
}
// buffer에서 문자 하나를 가져오는 함수
// Gp = Pp -> 버퍼가 비어있다. -> return 0
// Gp != Pp -> rxBuffer[rxBufferGp] 값을 가져오고, Gp++


int _write(int file, char *p, int len){
	HAL_UART_Transmit(myHuart, p, len, 10);
	return len;
}
