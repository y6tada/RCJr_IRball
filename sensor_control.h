#ifndef __SENSOR_CONTROL__
#define __SENSOR_CONTROL__

#include "Arduino.h"
#define IR_NUM 12

const uint8_t SensorPins[IR_NUM]  = {5, 6, 7, 8, 4, 2, 3, 17, 12, 11, 10, 9};
const float   unitVectorX[IR_NUM] = {0.000, 0.500, 0.866, 1.000, 0.866, 0.500, 0.000, -0.500, -0.866, -1.000, -0.866, -0.500};
const float   unitVectorY[IR_NUM] = {1.000, 0.866, 0.500, 0.000, -0.500, -0.866, -1.000, -0.866, -0.500, 0.000, 0.500, 0.866};
const float   deltaPulseWidth     = 2.0;

typedef struct {
  float x;
  float y;
  float degree;
  float distance;
} vector_t;

void setAllSensorPinsInput(void);
bool getSensorPin(uint8_t pin);
unsigned int getAllSensorPulseWidth(float *pulseWidth, const uint16_t timeLimit);
vector_t calcVectorFromPulseWidth(float *pulseWidth);
void convertCartToPolar(vector_t *self);

#endif
