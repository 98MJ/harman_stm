#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "main.h"
#include <stdbool.h>

#define maxValue	100

uint16_t movingAvgFilter(uint16_t); //선언시에는 변수형만 있어도 됨, 변수명 생략 가능
double Kalman(double);

#endif /* INC_FILTER_H_ */
