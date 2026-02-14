#include "stepper.h"
#include "heater.h"
#include <pins.h>

Stepper steppers[5];

const uint8_t stepperStepPins[5] = {
  EXTRUDER_1_MOTOR_DRIVER_STEP, EXTRUDER_2_MOTOR_DRIVER_STEP, EXTRUDER_3_MOTOR_DRIVER_STEP,
  EXTRUDER_4_MOTOR_DRIVER_STEP, EXTRUDER_5_MOTOR_DRIVER_STEP
};

const uint8_t stepperDirPins[5] = {
  EXTRUDER_1_MOTOR_DRIVER_DIR, EXTRUDER_2_MOTOR_DRIVER_DIR, EXTRUDER_3_MOTOR_DRIVER_DIR,
  EXTRUDER_4_MOTOR_DRIVER_DIR, EXTRUDER_5_MOTOR_DRIVER_DIR
};

const uint8_t stepperEnablePins[5] = {
  EXTRUDER_1_MOTOR_DRIVER_ENABLE, EXTRUDER_2_MOTOR_DRIVER_ENABLE, EXTRUDER_3_MOTOR_DRIVER_ENABLE,
  EXTRUDER_4_MOTOR_DRIVER_ENABLE, EXTRUDER_5_MOTOR_DRIVER_ENABLE
};

void setupSteppers() {
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    steppers[i].stepPin = stepperStepPins[i];
    steppers[i].dirPin = stepperDirPins[i];
    steppers[i].enablePin = stepperEnablePins[i];
    steppers[i].speed = DEFAULT_SPEED;
    steppers[i].enabled = false;
    steppers[i].stepState = false;
    steppers[i].lastStepTime = 0;
    
    pinMode(stepperStepPins[i], OUTPUT);
    pinMode(stepperDirPins[i], OUTPUT);
    pinMode(stepperEnablePins[i], OUTPUT);
    
    digitalWrite(stepperStepPins[i], LOW);
    digitalWrite(stepperDirPins[i], LOW);
    digitalWrite(stepperEnablePins[i], HIGH);
  }
}

void updateStepperStates() {
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    if (heaters[i].stable && heaters[i].enabled) {
      digitalWrite(stepperEnablePins[i], LOW);
      steppers[i].enabled = true;
    } else {
      digitalWrite(stepperEnablePins[i], HIGH);
      steppers[i].enabled = false;
    }
  }
}

void setMotorSpeed(uint8_t motorIndex, uint16_t speed) {
  if (motorIndex >= NUMBER_OF_HEATERS) {
    Serial.println("Invalid motor index. Use 0-4.");
    return;
  }
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  steppers[motorIndex].speed = speed;
  Serial.print("Motor ");
  Serial.print(motorIndex);
  Serial.print(" speed set to ");
  Serial.print(speed);
  Serial.println(" steps/sec");
}

void resetMotorSpeeds() {
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    steppers[i].speed = DEFAULT_SPEED;
  }
  Serial.println("All motor speeds reset to default (1000 steps/sec)");
}

void printMotorSpeeds() {
  Serial.println("\nMotor Speeds:");
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    Serial.print("  Motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(steppers[i].speed);
    Serial.println(" steps/sec");
  }
  Serial.println();
}

void printMotorStatus() {
  Serial.println("MOTORS:");
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    Serial.print("  Motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(steppers[i].speed);
    Serial.print(" steps/sec | Running: ");
    Serial.println(steppers[i].enabled ? "YES" : "NO ");
  }
}
