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

unsigned int getAllSensorPulseWidth(float *pulseWidth, uint16_t timeLimit) {
  
  for(int i = 0; i < IR_NUM; i++) {
    pulseWidth[i] = 0;
  }

  const unsigned long startTime_us = micros();
  do {
    for (int i = 0; i < IR_NUM; i++) {
      if(getSensorPin(i) == false) {
        pulseWidth[i] += deltaPulseWidth;
      }
    }
  } while((micros() - startTime_us) < timeLimit);

  uint8_t numOfReactiveSensor = 0;
  for(int i = 0; i < IR_NUM; i++) {
    if(pulseWidth[i]) numOfReactiveSensor++;
  }

  return numOfReactiveSensor;
}

vector_t calcVectorFromPulseWidth(float *pulseWidth) {
  vector_t rslt = {0, 0};
  for(int i = 0; i < IR_NUM; i++) {
    rslt.x += pulseWidth[i] * unitVectorX[i];
    rslt.y += pulseWidth[i] * unitVectorY[i];
  }
  
  return rslt;
}

void convertCartToPolar(vector_t *self) {
  self->distance  = sqrt(pow(self->x, 2.0) + pow(self->y, 2.0));
  self->degree    = atan2(self->x,self->y) / PI * 180.0;
}

