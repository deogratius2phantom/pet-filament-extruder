#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include "config.h"

struct Stepper {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enablePin;
  uint16_t speed;
  bool enabled;
  bool stepState;
  uint32_t lastStepTime;
};

extern Stepper steppers[5];
extern const uint8_t stepperStepPins[5];
extern const uint8_t stepperDirPins[5];
extern const uint8_t stepperEnablePins[5];

void setupSteppers();
void updateStepperStates();
void setMotorSpeed(uint8_t motorIndex, uint16_t speed);
void resetMotorSpeeds();
void printMotorSpeeds();
void printMotorStatus();

#endif // STEPPER_H
