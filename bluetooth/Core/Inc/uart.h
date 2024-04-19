#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
void initUART1(UART_HandleTypeDef *inHuart);
void initUART2(UART_HandleTypeDef *inHuart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t getChar1();
uint8_t getChar2();

#endif /* INC_UART_H_ */
