#include <Arduino.h>
#include "config.h"
#include "heater.h"
#include "stepper.h"
#include "encoder.h"
#include "commands.h"
#include <pins.h>
#include <thermistor.h>

// Rotary encoder interrupt handlers
ISR(INT3_vect) {
  if (!speedAdjustmentEnabled) return;
  
  uint8_t clkState = digitalRead(ENCODER_CLK);
  uint8_t dtState = digitalRead(ENCODER_DT);
  
  uint32_t currentTime = millis();
  uint32_t timeDiff = currentTime - lastEncoderTime;
  
  if (clkState != dtState) {
    int16_t increment;
    if (timeDiff < 50) {
      increment = 100;
    } else if (timeDiff < 200) {
      increment = 25;
    } else {
      increment = 5;
    }
    
    steppers[selectedMotorIndex].speed += increment;
    if (steppers[selectedMotorIndex].speed > MAX_SPEED) {
      steppers[selectedMotorIndex].speed = MAX_SPEED;
    }
  } else {
    uint16_t decrement;
    if (timeDiff < 50) {
      decrement = 100;
    } else if (timeDiff < 200) {
      decrement = 25;
    } else {
      decrement = 5;
    }
    
    if (steppers[selectedMotorIndex].speed > decrement) {
      steppers[selectedMotorIndex].speed -= decrement;
    } else {
      steppers[selectedMotorIndex].speed = MIN_SPEED;
    }
  }
  
  lastEncoderTime = currentTime;
}

ISR(INT1_vect) {
  if (!speedAdjustmentEnabled) return;
}

// Timer4 ISR - Heater control with software PWM
ISR(TIMER4_COMPA_vect) {
  pwmCycle++;
  if (pwmCycle >= PWM_PERIOD) {
    pwmCycle = 0;
  }
  
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    int adc = analogRead(heaters[i].sensorPin);
    heaters[i].input = getTemperature(adc);
    
    if (digitalRead(heaterEnableSwitchPins[i]) == LOW) {
      heaters[i].enabled = true;
      if (!thermalFault[i]) {
        heaters[i].pid->Compute();
        
        uint8_t scaledOutput = map(heaters[i].output, 0, 200, 0, PWM_PERIOD);
        
        if (pwmCycle < scaledOutput) {
          digitalWrite(heaters[i].pin, HIGH);
        } else {
          digitalWrite(heaters[i].pin, LOW);
        }
      } else {
        digitalWrite(heaters[i].pin, LOW);
      }
    } else {
      heaters[i].enabled = false;
      heaters[i].stable = false;
      digitalWrite(heaters[i].pin, LOW);
    }
  }
}

void setupTimer4() {
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  TCCR4B |= (1 << WGM42) | (1 << CS42) | (1 << CS40);
  OCR4A = 15624;
  TIMSK4 |= (1 << OCIE4A);
}

void setupTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  OCR1A = 999;
  TIMSK1 |= (1 << OCIE1A);
}

void setupTimer3() {
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  TCCR3B |= (1 << WGM32) | (1 << CS31);
  OCR3A = 999;
  TIMSK3 |= (1 << OCIE3A);
}

void setupTimer5() {
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  TCCR5B |= (1 << WGM52) | (1 << CS51);
  OCR5A = 999;
  TIMSK5 |= (1 << OCIE5A);
}

// Timer ISRs for stepper control
ISR(TIMER1_COMPA_vect) {
  static uint16_t counter[2] = {0, 0};
  const uint16_t BASE_FREQ = 2000;
  
  for (uint8_t i = 0; i < 2; i++) {
    if (steppers[i].enabled && steppers[i].speed > 0) {
      counter[i]++;
      uint16_t threshold = BASE_FREQ / steppers[i].speed;
      
      if (counter[i] >= threshold) {
        counter[i] = 0;
        steppers[i].stepState = !steppers[i].stepState;
        digitalWrite(stepperStepPins[i], steppers[i].stepState);
      }
    } else {
      counter[i] = 0;
      digitalWrite(stepperStepPins[i], LOW);
    }
  }
}

ISR(TIMER3_COMPA_vect) {
  static uint16_t counter[2] = {0, 0};
  const uint16_t BASE_FREQ = 2000;
  
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t stepperIdx = i + 2;
    if (steppers[stepperIdx].enabled && steppers[stepperIdx].speed > 0) {
      counter[i]++;
      uint16_t threshold = BASE_FREQ / steppers[stepperIdx].speed;
      
      if (counter[i] >= threshold) {
        counter[i] = 0;
        steppers[stepperIdx].stepState = !steppers[stepperIdx].stepState;
        digitalWrite(stepperStepPins[stepperIdx], steppers[stepperIdx].stepState);
      }
    } else {
      counter[i] = 0;
      digitalWrite(stepperStepPins[stepperIdx], LOW);
    }
  }
}

ISR(TIMER5_COMPA_vect) {
  static uint16_t counter = 0;
  const uint16_t BASE_FREQ = 2000;
  const uint8_t stepperIdx = 4;
  
  if (steppers[stepperIdx].enabled && steppers[stepperIdx].speed > 0) {
    counter++;
    uint16_t threshold = BASE_FREQ / steppers[stepperIdx].speed;
    
    if (counter >= threshold) {
      counter = 0;
      steppers[stepperIdx].stepState = !steppers[stepperIdx].stepState;
      digitalWrite(stepperStepPins[stepperIdx], steppers[stepperIdx].stepState);
    }
  } else {
    counter = 0;
    digitalWrite(stepperStepPins[stepperIdx], LOW);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize modules
  setupHeaters();
  loadAllPIDFromEEPROM();
  setupSteppers();
  setupEncoder();
  
  // Setup timers
  setupTimer1();
  setupTimer3();
  setupTimer5();
  setupTimer4();
  
  sei();
  
  Serial.println("PET Filament Extruder Controller Initialized");
  Serial.println("Software PWM enabled for all heaters (10Hz)");
  Serial.println("Use encoder to adjust speed, press button to select motor");
  Serial.print("Default speed: ");
  Serial.print(DEFAULT_SPEED);
  Serial.println(" steps/sec");
  Serial.println("\nType 'HELP' or 'MENU' for available commands\n");
}

void loop() {
  // Handle encoder button and LED blink
  handleEncoderButton();
  handleLEDBlink();
  
  // Update stepper and LED states
  updateStepperStates();
  updateHeaterLEDs();
  
  // Periodic status print
  static unsigned long lastPrint = 0;
  if (!statusUpdatesPaused && millis() - lastPrint > 1000) {
    lastPrint = millis();
    for (int i = 0; i < 5; i++) {
      Serial.print("Heater ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(heaters[i].input);
      Serial.print("C | Stable: ");
      Serial.print(heaters[i].stable ? "Yes" : "No");
      Serial.print(" | Enabled: ");
      Serial.println(heaters[i].enabled ? "Yes" : "No");
      checkThermalRunaway(i);
    }
    Serial.println();
  }
  
  // Resume status updates after timeout
  if (statusUpdatesPaused && (millis() - lastSerialInput) > STATUS_RESUME_TIMEOUT) {
    statusUpdatesPaused = false;
    Serial.println("[Status updates resumed]\n");
  }
  
  // Handle serial commands
  handleSerialCommands();
}
