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


// binary 방식으로 uart 통신

void binaryTransmit(protocol_t inData){
	uint8_t txBuffer[] = {STX, 0, 0, 0, 0, 0, 0, 0,ETX};
	//memcpy(&txBuffer[1], &inData, 6); // sizeof(protocol_t) = 6

	// STX, ETX를 오인하지 않기 위해, 최상위비트를 1로 설정
	txBuffer[1] = inData.id | 0x80;
	txBuffer[2] = inData.command | 0x80;
	txBuffer[3] = inData.data | 0x80;
	txBuffer[4] = (inData.data >> 7) | 0x80;
	txBuffer[5] = (inData.data >> 14) | 0x80;
	txBuffer[6] = (inData.data >> 21) | 0x80;
	for(int i = 0; i<7; i++){
		txBuffer[7] += txBuffer[i];
	}
	HAL_UART_Transmit(myHuart, txBuffer, sizeof(txBuffer), 10);
}

int _write(int file, char *p, int len){
	HAL_UART_Transmit(myHuart, p, len, 10);
	return len;
}
