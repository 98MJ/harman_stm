#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
void initUART(UART_HandleTypeDef *inHuart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t getChar();

#endif /* INC_UART_H_ */
