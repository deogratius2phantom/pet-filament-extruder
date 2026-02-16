#include "heater.h"
#include <pins.h>
#include <thermistor.h>
#include <EEPROM.h>

// Heater arrays
Heater heaters[5];
bool thermalFault[5] = {false, false, false, false, false};
bool softwareEnabled[5] = {true, true, true, true, true};
uint8_t pwmCycle = 0;

const uint8_t heaterPins[5] = {
  EXTRUDER_1_HEATER, EXTRUDER_2_HEATER, EXTRUDER_3_HEATER,
  EXTRUDER_4_HEATER, EXTRUDER_5_HEATER
};

const uint8_t thermistorPins[5] = {
  EXTRUDER_1_THERMISTOR, EXTRUDER_2_THERMISTOR, EXTRUDER_3_THERMISTOR,
  EXTRUDER_4_THERMISTOR, EXTRUDER_5_THERMISTOR
};

const uint8_t heaterEnableSwitchPins[5] = {
  EXTRUDER_1_ENABLE_SWITCH, EXTRUDER_2_ENABLE_SWITCH, EXTRUDER_3_ENABLE_SWITCH,
  EXTRUDER_4_ENABLE_SWITCH, EXTRUDER_5_ENABLE_SWITCH
};

const uint8_t heaterStableLEDPins[5] = {
  HEATER_1_STABLE_LED, HEATER_2_STABLE_LED, HEATER_3_STABLE_LED,
  HEATER_4_STABLE_LED, HEATER_5_STABLE_LED
};

void setupHeaters() {
  for (int i = 0; i < 5; i++) {
    heaters[i] = {heaterPins[i], thermistorPins[i], 0, 0, EXTRUSION_TEMPERATURE, 
                  nullptr, 0, 0, false, false};
    pinMode(heaterPins[i], OUTPUT);
    digitalWrite(heaterPins[i], LOW);
    pinMode(heaterEnableSwitchPins[i], INPUT_PULLUP);
    pinMode(heaterStableLEDPins[i], OUTPUT);
    digitalWrite(heaterStableLEDPins[i], LOW);
    
    heaters[i].pid = new PID(&heaters[i].input, &heaters[i].output, 
                             &heaters[i].setpoint, 38.4, 4.8, 40.7, DIRECT);
    heaters[i].pid->SetOutputLimits(0, 200);
    heaters[i].pid->SetMode(AUTOMATIC);
  }
}

void updateHeaterLEDs() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(heaterStableLEDPins[i], heaters[i].stable ? HIGH : LOW);
  }
}

void checkThermalRunaway(uint8_t i) {
  if (!heaters[i].enabled || thermalFault[i]) return;
  
  float currentTemp = heaters[i].input;
  if (currentTemp < heaters[i].lastTemp + 1 && 
      (millis() - heaters[i].lastUpdate) > TIMEOUT_MS) {
    digitalWrite(heaters[i].pin, LOW);
    thermalFault[i] = true;
    Serial.print("Thermal runaway detected on heater ");
    Serial.println(i);
  }
  
  if (abs(currentTemp - heaters[i].setpoint) < 2) {
    heaters[i].stable = true;
  }
  
  heaters[i].lastTemp = currentTemp;
  heaters[i].lastUpdate = millis();
}

#define EEPROM_PID_ADDR_BASE 0
#define EEPROM_PID_BLOCK_SIZE 12

void savePIDToEEPROM(uint8_t heaterIndex, double Kp, double Ki, double Kd) {
  int addr = EEPROM_PID_ADDR_BASE + heaterIndex * EEPROM_PID_BLOCK_SIZE;
  EEPROM.put(addr, Kp); addr += sizeof(float);
  EEPROM.put(addr, Ki); addr += sizeof(float);
  EEPROM.put(addr, Kd);
}

void loadPIDFromEEPROM(uint8_t heaterIndex, double &Kp, double &Ki, double &Kd) {
  int addr = EEPROM_PID_ADDR_BASE + heaterIndex * EEPROM_PID_BLOCK_SIZE;
  EEPROM.get(addr, Kp); addr += sizeof(float);
  EEPROM.get(addr, Ki); addr += sizeof(float);
  EEPROM.get(addr, Kd);
}

void loadAllPIDFromEEPROM() {
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    double Kp, Ki, Kd;
    loadPIDFromEEPROM(i, Kp, Ki, Kd);
    if (Kp > 0 && Ki > 0 && Kd > 0) {
      heaters[i].pid->SetTunings(Kp, Ki, Kd);
    }
  }
}

void printPIDValues(uint8_t heaterIndex) {
  if (heaterIndex >= NUMBER_OF_HEATERS) {
    Serial.println("Invalid heater index. Use 0-4.");
    return;
  }
  double Kp = heaters[heaterIndex].pid->GetKp();
  double Ki = heaters[heaterIndex].pid->GetKi();
  double Kd = heaters[heaterIndex].pid->GetKd();
  
  Serial.print("Heater ");
  Serial.print(heaterIndex);
  Serial.println(" PID Values:");
  Serial.print("  Kp: "); Serial.println(Kp, 4);
  Serial.print("  Ki: "); Serial.println(Ki, 4);
  Serial.print("  Kd: "); Serial.println(Kd, 4);
}

void printAllPIDValues() {
  Serial.println("\n========== ALL PID VALUES ==========");
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    double Kp = heaters[i].pid->GetKp();
    double Ki = heaters[i].pid->GetKi();
    double Kd = heaters[i].pid->GetKd();
    
    Serial.print("Heater ");
    Serial.print(i);
    Serial.print(": Kp=");
    Serial.print(Kp, 4);
    Serial.print(" Ki=");
    Serial.print(Ki, 4);
    Serial.print(" Kd=");
    Serial.println(Kd, 4);
  }
  Serial.println("====================================\n");
}

void setPIDValues(uint8_t heaterIndex, double Kp, double Ki, double Kd) {
  if (heaterIndex >= NUMBER_OF_HEATERS) {
    Serial.println("Invalid heater index. Use 0-4.");
    return;
  }
  heaters[heaterIndex].pid->SetTunings(Kp, Ki, Kd);
  savePIDToEEPROM(heaterIndex, Kp, Ki, Kd);
  Serial.print("Heater ");
  Serial.print(heaterIndex);
  Serial.println(" PID values updated and saved.");
  printPIDValues(heaterIndex);
}

void printDetailedStatus() {
  Serial.println("\n========== SYSTEM STATUS ==========");
  Serial.println("HEATERS:");
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    Serial.print("  Heater ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(heaters[i].input, 1);
    Serial.print("C / ");
    Serial.print(heaters[i].setpoint, 0);
    Serial.print("C | Output: ");
    Serial.print(heaters[i].output, 0);
    Serial.print(" | Stable: ");
    Serial.print(heaters[i].stable ? "YES" : "NO ");
    Serial.print(" | Enabled: ");
    Serial.print(heaters[i].enabled ? "YES" : "NO ");
    Serial.print(" | SW: ");
    Serial.print(softwareEnabled[i] ? "ON " : "OFF");
    Serial.print(" | HW: ");
    Serial.print(digitalRead(heaterEnableSwitchPins[i]) == LOW ? "ON " : "OFF");
    Serial.print(" | Fault: ");
    Serial.println(thermalFault[i] ? "YES" : "NO");
  }
  Serial.println("===================================\n");
}

void enableHeater(uint8_t heaterIndex) {
  if (heaterIndex >= NUMBER_OF_HEATERS) {
    Serial.println("Invalid heater index. Use 0-4.");
    return;
  }
  softwareEnabled[heaterIndex] = true;
  Serial.print("Heater ");
  Serial.print(heaterIndex);
  Serial.println(" enabled via software.");
}

void disableHeater(uint8_t heaterIndex) {
  if (heaterIndex >= NUMBER_OF_HEATERS) {
    Serial.println("Invalid heater index. Use 0-4.");
    return;
  }
  softwareEnabled[heaterIndex] = false;
  heaters[heaterIndex].enabled = false;
  heaters[heaterIndex].stable = false;
  digitalWrite(heaters[heaterIndex].pin, LOW);
  Serial.print("Heater ");
  Serial.print(heaterIndex);
  Serial.println(" disabled via software.");
}

void enableAllHeaters() {
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    softwareEnabled[i] = true;
  }
  Serial.println("All heaters enabled via software.");
}

void disableAllHeaters() {
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    softwareEnabled[i] = false;
    heaters[i].enabled = false;
    heaters[i].stable = false;
    digitalWrite(heaters[i].pin, LOW);
  }
  Serial.println("All heaters disabled via software.");
}

void autoTunePID(uint8_t heaterIndex, double tuneSetpoint, int power, int cycles) {
    Serial.print("Manual PID Autotune start: Heater "); Serial.print(heaterIndex);
    Serial.print(" | Setpoint: "); Serial.print(tuneSetpoint);
    Serial.print("C | Power: "); Serial.print(power);
    Serial.print(" | Cycles: "); Serial.println(cycles);

    heaters[heaterIndex].setpoint = tuneSetpoint;
    double input = getTemperature(analogRead(heaters[heaterIndex].sensorPin));
    double output = 0;
    bool heating = true;
    int cycleCount = 0;
    double maxTemp = input, minTemp = input;
    unsigned long t1 = 0, t2 = 0, period = 0;
    double Ku = 0, Pu = 0;
    double lastInput = input;
    unsigned long start = millis();
    unsigned long lastUpdate = millis();

    digitalWrite(heaters[heaterIndex].pin, LOW);
    delay(500);

    while (cycleCount < cycles) {
        input = getTemperature(analogRead(heaters[heaterIndex].sensorPin));

        Serial.print("Tuning Heater ");
        Serial.print(heaterIndex);
        Serial.print(" | Temp: ");
        Serial.print(input, 1);
        Serial.print(" C | Output: ");
        Serial.println(output);

        if (input < lastInput + 1 && (millis() - lastUpdate) > TIMEOUT_MS) {
            digitalWrite(heaters[heaterIndex].pin, LOW);
            Serial.println("Thermal runaway detected during auto-tune. Aborting.");
            return;
        }
        lastInput = input;
        lastUpdate = millis();

        if (millis() - start > AUTOTUNE_TIMEOUT_MS) {
            digitalWrite(heaters[heaterIndex].pin, LOW);
            Serial.println("Auto-tune timeout.");
            return;
        }

        if (heating) {
            output = power;
            digitalWrite(heaters[heaterIndex].pin, output > 0 ? HIGH : LOW);
            if (input > tuneSetpoint + 5) {
                heating = false;
                maxTemp = input;
                t1 = millis();
            }
        } else {
            output = 0;
            digitalWrite(heaters[heaterIndex].pin, LOW);
            if (input < tuneSetpoint - 5) {
                heating = true;
                minTemp = input;
                t2 = millis();
                period = t2 - t1;
                t1 = t2;
                cycleCount++;
                Serial.print("Cycle "); Serial.print(cycleCount);
                Serial.print(": Max="); Serial.print(maxTemp, 2);
                Serial.print(" Min="); Serial.print(minTemp, 2);
                Serial.print(" Period="); Serial.print(period / 1000.0, 2); Serial.println("s");
                Ku = (4.0 * power) / (3.14159 * (maxTemp - minTemp));
                Pu = period / 1000.0;
            }
        }
        delay(100);
    }
    digitalWrite(heaters[heaterIndex].pin, LOW);

    double Kp = 0.6 * Ku;
    double Ki = 1.2 * Ku / Pu;
    double Kd = 0.075 * Ku * Pu;

    Serial.println("Manual PID Autotune finished!");
    Serial.print("Kp: "); Serial.print(Kp, 4);
    Serial.print(" Ki: "); Serial.print(Ki, 4);
    Serial.print(" Kd: "); Serial.println(Kd, 4);

    savePIDToEEPROM(heaterIndex, Kp, Ki, Kd);
    heaters[heaterIndex].pid->SetTunings(Kp, Ki, Kd);
}
