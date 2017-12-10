#include "sensor_control.h"
#include "moving_average.h"

// RoboCupJunior IR Ball waveform MODE-A T=833[us]
// https://www.elekit.co.jp/pdf/RCJ-05%20waveform_j.pdf
#define T_MODEA 833

MovingAverage smaForRadius(20);
MovingAverage smaForTheta(20);

unsigned long time_ms = 0;

void setup() {
	Serial.begin(115200);
	setAllSensorPinsInput();
}

void loop() {
	uint8_t 		activeSensors;			// 反応したセンサの個数
	uint16_t 		maxPulseWidth;			// 最大のセンサ値
	uint8_t 		maxSensorNumber;		// 最大の値を観測したセンサの番号
	float 			pulseWidth[IR_NUM];	// パルス幅を格納する変数
	vectorXY_t 	vectorXY;						// 直交座標系のベクトル構造体
	vectorRT_t	vectorRT;						// 極座標系のベクトル構造体

	getAllSensorPulseWidth(&activeSensors, &maxPulseWidth, &maxSensorNumber, pulseWidth, T_MODEA);
	vectorXY = calcVectorXYFromPulseWidth(pulseWidth);
	vectorRT = calcRTfromXY(&vectorXY);

	vectorRT_t vectorRTWithSma; 
	vectorRTWithSma.theta 	= smaForTheta.updateData(vectorRT.theta);
	vectorRTWithSma.radius 	= smaForRadius.updateData(vectorRT.radius);

	// 50ms周期でシリアルプリント
	if (millis() - time_ms > 50) {
		time_ms = millis();
		
		serialPrintAllPusleWidth(pulseWidth, activeSensors, maxSensorNumber, maxPulseWidth);
		Serial.print("\t");
		serialPrintVectorXY(&vectorXY);
		Serial.print("\t");
		serialPrintVectorRT(&vectorRTWithSma);
		Serial.print("\t");
		Serial.print(millis());
		Serial.print("\n");
	}
}

void serialPrintAllPusleWidth(float *pulseWidth, uint8_t activeSensors, uint8_t maxSensorNumber, uint16_t maxPulseWidth) {
	for(int i = 0; i < IR_NUM; i++) {
		Serial.print(pulseWidth[i]); 
		Serial.print("\t");
	}
	Serial.print(activeSensors); 
	Serial.print("\t");
	Serial.print(maxSensorNumber);
	Serial.print("\t");
	Serial.print(maxPulseWidth); 
}

void serialPrintVectorXY(vectorXY_t *self) {
	Serial.print(self->x); 
	Serial.print("\t");
	Serial.print(self->y);
}

void serialPrintVectorRT(vectorRT_t *self) {
	Serial.print(self->radius);
	Serial.print("\t");
	Serial.print(self->theta);
}


