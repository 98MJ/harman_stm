#include "filter.h"


uint16_t movingAvgFilter(uint16_t inData){//이동 평균 필터
	static uint16_t filterBuffer[100];
	static uint32_t sumValue = 0;
	static uint16_t bufpos = 0;
	static _Bool	isFirst = 0;
	if(isFirst == 0){
		isFirst = 1;
		for(int i=0; i<maxValue; i++){
			filterBuffer[i] = inData;
		}
	}
	// 합계(sumValue)에서 현재 위치(bufpos)의 버퍼값(filterBuffer) 차감
	sumValue -= filterBuffer[bufpos];
	// 현재 위치의 버퍼값 갱신
	filterBuffer[bufpos] = inData;
	// 합계에 현재 위치의 버퍼값 더하기
	sumValue += filterBuffer[bufpos];
	// 현재 위치 증가
	bufpos++;
	// bufpos가 100이상이 되면 리셋
	bufpos %= maxValue;

	return sumValue / maxValue;
}


double Kalman(double measurement){
	static double P = 1.0; 					// 추정 오차의 공분산
	static double varP = 0.0001;			// 프로세스 변동성
	static double R = 0.25;//pow(0.5, 2);	// 측정치 오차의 공분산
	static double K = 1.0;					// 칼만 이득
	static double X = 20.0;					// 현재 추정된 상태

	// Kalman Simple Filter
	P = P + varP;
	K = P / (P + R);
	X = (K * measurement) + (1 - K) * X;
	P = (1 - K) * P;
	return X;

}
