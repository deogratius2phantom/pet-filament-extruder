#include <Arduino.h>
#include "config.h"
#include "heater.h"
#include "stepper.h"
#include "commands.h"
#include <pins.h>
#include <thermistor.h>

// Timer4 ISR - Heater control with software PWM
ISR(TIMER4_COMPA_vect) {
  // Increment PWM cycle counter
  pwmCycle++;
  if (pwmCycle >= PWM_PERIOD) {
    pwmCycle = 0;
  }
  
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    // Read temperature and update PID
    int adc = analogRead(heaters[i].sensorPin);
    heaters[i].input = getTemperature(adc);
    
    bool hardwareEnabled = (digitalRead(heaterEnableSwitchPins[i]) == LOW);
    bool shouldBeEnabled = hardwareEnabled && softwareEnabled[i];
    
    // Detect transition from disabled to enabled
    if (shouldBeEnabled && !heaters[i].enabled) {
      // Re-enabling heater - reset thermal monitoring state
      heaters[i].lastTemp = heaters[i].input;
      heaters[i].lastUpdate = millis();
      thermalFault[i] = false; // Clear any previous thermal fault
      heaters[i].stable = false; // Reset stability flag
    }
    
    // Heater is enabled only if BOTH software AND hardware enable it
    if (shouldBeEnabled) {
      heaters[i].enabled = true;
      if (!thermalFault[i]) {
        heaters[i].pid->Compute();
        
        // Software PWM implementation
        // Map PID output (0-200) to PWM cycles (0-10)
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
      heaters[i].stable = false;  // Reset stability when disabled
      digitalWrite(heaters[i].pin, LOW); // Turn off heater if not enabled
    }
  }
}

// Timer1 ISR - Handles steppers 0 and 1
ISR(TIMER1_COMPA_vect) {
  static uint16_t counter[2] = {0, 0};
  const uint16_t BASE_FREQ = 2000; // 2kHz base frequency
  
  for (uint8_t i = 0; i < 2; i++) {
    if (steppers[i].enabled && steppers[i].speed > 0) {
      counter[i]++;
      uint16_t threshold = BASE_FREQ / steppers[i].speed;
      
      if (counter[i] >= threshold) {
        counter[i] = 0;
        // Toggle step pin
        steppers[i].stepState = !steppers[i].stepState;
        digitalWrite(stepperStepPins[i], steppers[i].stepState);
      }
    } else {
      counter[i] = 0;
      digitalWrite(stepperStepPins[i], LOW);
    }
  }
}

// Timer3 ISR - Handles steppers 2 and 3
ISR(TIMER3_COMPA_vect) {
  static uint16_t counter[2] = {0, 0};
  const uint16_t BASE_FREQ = 2000; // 2kHz base frequency
  
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t stepperIdx = i + 2;
    if (steppers[stepperIdx].enabled && steppers[stepperIdx].speed > 0) {
      counter[i]++;
      uint16_t threshold = BASE_FREQ / steppers[stepperIdx].speed;
      
      if (counter[i] >= threshold) {
        counter[i] = 0;
        // Toggle step pin
        steppers[stepperIdx].stepState = !steppers[stepperIdx].stepState;
        digitalWrite(stepperStepPins[stepperIdx], steppers[stepperIdx].stepState);
      }
    } else {
      counter[i] = 0;
      digitalWrite(stepperStepPins[stepperIdx], LOW);
    }
  }
}

// Timer5 ISR - Handles stepper 4
ISR(TIMER5_COMPA_vect) {
  static uint16_t counter = 0;
  const uint16_t BASE_FREQ = 2000; // 2kHz base frequency
  const uint8_t stepperIdx = 4;
  
  if (steppers[stepperIdx].enabled && steppers[stepperIdx].speed > 0) {
    counter++;
    uint16_t threshold = BASE_FREQ / steppers[stepperIdx].speed;
    
    if (counter >= threshold) {
      counter = 0;
      // Toggle step pin
      steppers[stepperIdx].stepState = !steppers[stepperIdx].stepState;
      digitalWrite(stepperStepPins[stepperIdx], steppers[stepperIdx].stepState);
    }
  } else {
    counter = 0;
    digitalWrite(stepperStepPins[stepperIdx], LOW);
  }
}

// Timer setup functions
void setupTimer4() {
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  TCCR4B |= (1 << WGM42) | (1 << CS42) | (1 << CS40); // CTC mode, prescaler 1024
  OCR4A = 15624; // 100 ms at 16 MHz
  TIMSK4 |= (1 << OCIE4A); // Enable interrupt
}

void setupTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8
  OCR1A = 999; // ~2kHz at 16MHz (16MHz / 8 / 2000 = 1000)
  TIMSK1 |= (1 << OCIE1A); // Enable interrupt
}

void setupTimer3() {
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  TCCR3B |= (1 << WGM32) | (1 << CS31); // CTC mode, prescaler 8
  OCR3A = 999; // ~2kHz at 16MHz
  TIMSK3 |= (1 << OCIE3A); // Enable interrupt
}

void setupTimer5() {
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  TCCR5B |= (1 << WGM52) | (1 << CS51); // CTC mode, prescaler 8
  OCR5A = 999; // ~2kHz at 16MHz
  TIMSK5 |= (1 << OCIE5A); // Enable interrupt
}

void setup() {
  Serial.begin(115200);
  
  // Initialize all subsystems
  setupHeaters();
  loadAllPIDFromEEPROM();
  setupSteppers();
  
  // Setup all timers
  setupTimer1();  // Steppers 0-1
  setupTimer3();  // Steppers 2-3
  setupTimer5();  // Stepper 4
  setupTimer4();  // Heater control
  
  sei(); // Enable interrupts
  
  Serial.println("PET Filament Extruder Controller Initialized");
  Serial.println("Software PWM enabled for all heaters (10Hz)");
  Serial.print("Default speed: ");
  Serial.print(DEFAULT_SPEED);
  Serial.println(" steps/sec");
  Serial.println("\nType 'HELP' or 'MENU' for available commands\n");
}

void loop() {

  // Update stepper enable states based on heater status
  updateStepperStates();
  
  // Print periodic status updates
  static unsigned long lastPrint = 0;
  if (!statusUpdatesPaused && millis() - lastPrint > 1000) {
    lastPrint = millis();
    for (int i = 0; i < 5; i++) {
      Serial.print("Heater ");
      Serial.print(i + 1);
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
  
  // Resume status updates after 15 seconds of no serial input
  if (statusUpdatesPaused && (millis() - lastSerialInput) > STATUS_RESUME_TIMEOUT) {
    statusUpdatesPaused = false;
    Serial.println("[Status updates resumed]\n");
  }
  
  // Handle serial commands
  handleSerialCommands();
}
