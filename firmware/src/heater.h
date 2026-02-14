#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"

struct Heater {
  uint8_t pin;
  uint8_t sensorPin;
  double input, output, setpoint;
  PID* pid;
  float lastTemp;
  uint32_t lastUpdate;
  bool enabled;
  bool stable;
};

// Global heater arrays
extern Heater heaters[5];
extern const uint8_t heaterPins[5];
extern const uint8_t thermistorPins[5];
extern const uint8_t heaterEnableSwitchPins[5];
extern const uint8_t heaterStableLEDPins[5];
extern bool thermalFault[5];
extern uint8_t pwmCycle;

// Functions
void setupHeaters();
void updateHeaterLEDs();
void checkThermalRunaway(uint8_t i);
void savePIDToEEPROM(uint8_t heaterIndex, double Kp, double Ki, double Kd);
void loadPIDFromEEPROM(uint8_t heaterIndex, double &Kp, double &Ki, double &Kd);
void loadAllPIDFromEEPROM();
void autoTunePID(uint8_t heaterIndex, double tuneSetpoint, int power, int cycles);
void printPIDValues(uint8_t heaterIndex);
void printAllPIDValues();
void setPIDValues(uint8_t heaterIndex, double Kp, double Ki, double Kd);
void printDetailedStatus();

#endif // HEATER_H
