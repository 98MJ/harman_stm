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


int16_t getChar(){
	int8_t result;
	if(rxBufferGp == rxBufferPp) return -1;
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

void transmitPacket(protocol_t data){
	uint8_t txBuffer[] = {STX, 0, 0, 0, 0, ETX};
	txBuffer[1] = data.command;
	txBuffer[2] = (data.data >> 7) | 0x80;
	txBuffer[3] = (data.data & 0x7f) | 0x80;
	txBuffer[4] = txBuffer[0] + txBuffer[1]+ txBuffer[2]+ txBuffer[3];

	HAL_UART_Transmit(myHuart, txBuffer, sizeof(txBuffer), 1);

	while(HAL_UART_GetState(myHuart) == HAL_UART_STATE_BUSY_TX
			|| HAL_UART_GetState(myHuart) == HAL_UART_STATE_BUSY_TX_RX);
}

protocol_t receivePacket(){
	protocol_t result;
	uint8_t	buffer[6];
	uint8_t count = 0;
	uint32_t timeout;

	int16_t ch = getChar();
	memset(&result, 0, sizeof(buffer));

	if(ch == STX){
		buffer[count++] = ch;
		timeout = HAL_GetTick();
		while(ch != ETX){
			ch = getChar();
			if(ch != -1){ //수신된 데이터가 있다면
				buffer[count++] = ch;
			}
		if(HAL_GetTick() - timeout >= 2) return result;
		}

		//CRC 검사
		uint8_t crc = 0;
		for(int i=0; i<4; i++){
			crc+= buffer[i];
		}
		if(crc != buffer[4]) return result;

		// 수신완료 후 파싱
		result.command = buffer[1];
		result.data = buffer[3] & 0x7f;
		result.data |= (buffer[2] & 0x7f) <<7;
	}
	return result;
}
