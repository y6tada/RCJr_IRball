#include "sensor_control.h"
#include "moving_average.h"

MovingAverage smaForRadius(20);
MovingAverage smaForTheta(20);

static unsigned long time_ms = 0;

void setup() {
// put your setup code here, to run once:
	Serial.begin(500000);
	setAllSensorPinsInput();
}

void loop() {
// put your main code here, to run repeatedly:
	uint8_t 		activeSensors;
	uint16_t 		maxPulseWidth;
	uint8_t 		maxSensorNumber;
	float 			pulseWidth[IR_NUM];
	vectorXY_t 	vectorXY;
	vectorRT_t	vectorRT;

	getAllSensorPulseWidth(&activeSensors, &maxPulseWidth, &maxSensorNumber, pulseWidth, 833);
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


