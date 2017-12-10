#include "sensor_control.h"

void setAllSensorPinsInput(void) {
	for (int i = 0; i < IR_NUM; ++i) {
		pinMode(SensorPins[i], INPUT);
	}
}

bool getSensorPin(uint8_t pin) {
	switch(pin) {
		case 0:   return PINB&(1<<2);
		case 1:   return PINB&(1<<1);
		case 2:   return PIND&(1<<5);
		case 3:   return PIND&(1<<6);
		case 4:   return PIND&(1<<7);
		case 5:   return PINB&(1<<0);
		case 6:   return PIND&(1<<4);
		case 7:   return PIND&(1<<2);
		case 8:   return PIND&(1<<3);
		case 9:   return PINC&(1<<3);
		case 10:  return PINB&(1<<4);
		case 11:  return PINB&(1<<3);
	}
}

void getAllSensorPulseWidth(uint8_t *activeSensors_p, uint16_t *maxPulseWidth_p, uint8_t *maxSensorNumber_p, float *pulseWidth, uint16_t timeLimit) {
	// pulseWidth[]は加算計算用変数なので最初に初期化する
	for(int i = 0; i < IR_NUM; i++) {
		pulseWidth[i] = 0;
	}

	// do-whileで時間(833us)を監視しながらセンサの読み込み
	const unsigned long startTime_us = micros();
	do {
		for (int i = 0; i < IR_NUM; i++) {
			if(getSensorPin(i) == false) {
				pulseWidth[i] += deltaPulseWidth;
			}
		}
	} while((micros() - startTime_us) < timeLimit);

	*activeSensors_p 		= 0;	// ボールに反応しているセンサの個数
	*maxPulseWidth_p 		= 0;	// 一番反応の強いセンサのパルス幅
	*maxSensorNumber_p 	= 0;	// 一番反応の強いセンサの番号
	for(int i = 0; i < IR_NUM; i++) {
		if(pulseWidth[i] > 0) {
			*activeSensors_p += 1;
		}
		if(pulseWidth[i] > *maxPulseWidth_p) {
			*maxPulseWidth_p = pulseWidth[i];
			*maxSensorNumber_p = i;
		}
	}
}

vectorXY_t calcVectorXYFromPulseWidth(float *pulseWidth) {
	vectorXY_t rslt = {0, 0};
	for(int i = 0; i < IR_NUM; i++) {
		rslt.x += pulseWidth[i] * unitVectorX[i];
		rslt.y += pulseWidth[i] * unitVectorY[i];
	}

	return rslt;
}

vectorRT_t calcRTfromXY(vectorXY_t *vectorXY_p) {
	vectorRT_t rslt;
	rslt.radius  = sqrt(pow(vectorXY_p->x, 2.0) + pow(vectorXY_p->y, 2.0));
	rslt.theta   = atan2(vectorXY_p->x, vectorXY_p->y) / PI * 180.0;

	return rslt;
}
