#include <Arduino.h>
#include <pins.h>
#include <thermistor.h>
#include <PID_v1.h> // Include the PID library header
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>
#define EXRTUSION_TEMPERATURE 220 // Set the target temperature for the extruder for PET 220 to 235 degrees
const uint8_t NUMBER_OF_HEATERS = 5; // Number of heaters
// put function declarations here:
struct Heater {
  uint8_t pin;                   // Pin connected to the heater mosfet gate 
  uint8_t sensorPin;             // ADC Pin connected to the thermistor   
  double input, output, setpoint;// PID variables for corresponding heater
  PID* pid;                      // PID object for the heater
  float lastTemp;                // Last temperature reading
  uint32_t lastUpdate;          // Last update time for the heater
  bool enabled;                 // Flag to indicate if the heater is enabled
  bool stable;                  // Flag to indicate if the heater is stable
};

Heater heaters[NUMBER_OF_HEATERS];  //Array of 5 heaters
const uint8_t heaterPins[5] = {
  EXTRUDER_1_HEATER, 
  EXTRUDER_2_HEATER, 
  EXTRUDER_3_HEATER, 
  EXTRUDER_4_HEATER, 
  EXTRUDER_5_HEATER
};

const uint8_t thermistorPins[5] = {
  EXTRUDER_1_THERMISTOR, 
  EXTRUDER_2_THERMISTOR, 
  EXTRUDER_3_THERMISTOR, 
  EXTRUDER_4_THERMISTOR, 
  EXTRUDER_5_THERMISTOR
};

const uint8_t heaterEnableSwitchPins[5] = {
  EXTRUDER_1_ENABLE_SWITCH, 
  EXTRUDER_2_ENABLE_SWITCH, 
  EXTRUDER_3_ENABLE_SWITCH, 
  EXTRUDER_4_ENABLE_SWITCH, 
  EXTRUDER_5_ENABLE_SWITCH
};

const uint8_t heaterStableLEDPins[5] = {
  HEATER_1_STABLE_LED,
  HEATER_2_STABLE_LED,
  HEATER_3_STABLE_LED,
  HEATER_4_STABLE_LED,
  HEATER_5_STABLE_LED
};

bool thermalFault[5] = {false, false, false, false, false};  // Array to store thermal fault status for each heater
const uint32_t TIMEOUT_MS = 20000; // Timeout duration in milliseconds
const uint32_t AUTOTUNE_TIMEOUT_MS = 600000; // 10 minutes
const int AUTOTUNE_CYCLES = 5; // Marlin default is 8 cycles

// Software PWM for heaters
const uint8_t PWM_PERIOD = 10; // 10 cycles = 1 second (10 * 100ms)
uint8_t pwmCycle = 0;

// Stepper motor structure
struct Stepper {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enablePin;
  uint16_t speed;           // Steps per second
  bool enabled;
  bool stepState;           // Current state of step pin for toggling
  uint32_t lastStepTime;    // Microseconds timestamp for last step
};

Stepper steppers[NUMBER_OF_HEATERS];  // Array of 5 stepper motors

const uint8_t stepperStepPins[5] = {
  EXTRUDER_1_MOTOR_DRIVER_STEP, 
  EXTRUDER_2_MOTOR_DRIVER_STEP, 
  EXTRUDER_3_MOTOR_DRIVER_STEP, 
  EXTRUDER_4_MOTOR_DRIVER_STEP, 
  EXTRUDER_5_MOTOR_DRIVER_STEP
};

const uint8_t stepperDirPins[5] = {
  EXTRUDER_1_MOTOR_DRIVER_DIR, 
  EXTRUDER_2_MOTOR_DRIVER_DIR, 
  EXTRUDER_3_MOTOR_DRIVER_DIR, 
  EXTRUDER_4_MOTOR_DRIVER_DIR, 
  EXTRUDER_5_MOTOR_DRIVER_DIR
};

const uint8_t stepperEnablePins[5] = {
  EXTRUDER_1_MOTOR_DRIVER_ENABLE, 
  EXTRUDER_2_MOTOR_DRIVER_ENABLE, 
  EXTRUDER_3_MOTOR_DRIVER_ENABLE, 
  EXTRUDER_4_MOTOR_DRIVER_ENABLE, 
  EXTRUDER_5_MOTOR_DRIVER_ENABLE
};

// Rotary encoder variables
volatile int8_t selectedMotorIndex = 0;      // Currently selected motor (0-4)
volatile int encoderPos = 0;                 // Encoder position counter
volatile uint8_t lastEncoderState = 0;       // Last state of encoder pins
volatile uint32_t lastEncoderTime = 0;       // Time of last encoder pulse for adaptive scaling
volatile bool speedAdjustmentEnabled = true; // Disabled during LED blink

// Button debouncing
uint8_t lastButtonState = HIGH;
uint8_t buttonState = HIGH;
uint32_t lastButtonPress = 0;
const uint32_t DEBOUNCE_DELAY = 50;

// LED blink state machine
bool isBlinking = false;
uint8_t blinkCount = 0;
uint8_t currentBlink = 0;
bool ledState = false;
uint32_t lastBlinkTime = 0;
const uint32_t BLINK_INTERVAL = 200;  // 200ms on/off

// Speed limits
const uint16_t MIN_SPEED = 50;    // Minimum steps/sec
const uint16_t MAX_SPEED = 3000;  // Maximum steps/sec
const uint16_t DEFAULT_SPEED = 1000; // Default steps/sec

// Status update control
bool statusUpdatesPaused = false;
uint32_t lastSerialInput = 0;
const uint32_t STATUS_RESUME_TIMEOUT = 15000; // 15 seconds

void checkThermalRunaway(uint8_t i) {
  if (!heaters[i].enabled || thermalFault[i]) return;
  float currentTemp = heaters[i].input;
  if (currentTemp < heaters[i].lastTemp + 1 && (millis() - heaters[i].lastUpdate) > TIMEOUT_MS) {
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

// Rotary encoder interrupt handlers
ISR(INT3_vect) {
  // Encoder CLK interrupt (pin 18)
  if (!speedAdjustmentEnabled) return; // Disabled during LED blink
  
  uint8_t clkState = digitalRead(ENCODER_CLK);
  uint8_t dtState = digitalRead(ENCODER_DT);
  
  uint32_t currentTime = millis();
  uint32_t timeDiff = currentTime - lastEncoderTime;
  
  // Determine direction
  if (clkState != dtState) {
    // Clockwise rotation - increase speed
    int16_t increment;
    if (timeDiff < 50) {
      increment = 100; // Fast rotation
    } else if (timeDiff < 200) {
      increment = 25;  // Medium rotation
    } else {
      increment = 5;   // Slow rotation
    }
    
    steppers[selectedMotorIndex].speed += increment;
    if (steppers[selectedMotorIndex].speed > MAX_SPEED) {
      steppers[selectedMotorIndex].speed = MAX_SPEED;
    }
  } else {
    // Counter-clockwise rotation - decrease speed
    uint16_t decrement;
    if (timeDiff < 50) {
      decrement = 100; // Fast rotation
    } else if (timeDiff < 200) {
      decrement = 25;  // Medium rotation
    } else {
      decrement = 5;   // Slow rotation
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
  // Encoder DT interrupt (pin 20) - secondary for better responsiveness
  if (!speedAdjustmentEnabled) return;
}

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
    
    if (digitalRead(heaterEnableSwitchPins[i]) == LOW) {
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

void setupTimer4() {
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  TCCR4B |= (1 << WGM42) | (1 << CS42) | (1 << CS40); // CTC mode, prescaler 1024
  OCR4A = 15624; // 100 ms at 16 MHz
  TIMSK4 |= (1 << OCIE4A); // Enable interrupt
}

// Timer1 setup for steppers 0 and 1
void setupTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8
  OCR1A = 999; // ~2kHz at 16MHz (16MHz / 8 / 2000 = 1000)
  TIMSK1 |= (1 << OCIE1A); // Enable interrupt
}

// Timer3 setup for steppers 2 and 3
void setupTimer3() {
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  TCCR3B |= (1 << WGM32) | (1 << CS31); // CTC mode, prescaler 8
  OCR3A = 999; // ~2kHz at 16MHz
  TIMSK3 |= (1 << OCIE3A); // Enable interrupt
}

// Timer5 setup for stepper 4
void setupTimer5() {
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  TCCR5B |= (1 << WGM52) | (1 << CS51); // CTC mode, prescaler 8
  OCR5A = 999; // ~2kHz at 16MHz
  TIMSK5 |= (1 << OCIE5A); // Enable interrupt
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

void setupHeaters() {
  for (int i = 0; i < 5; i++) {
    heaters[i] = {heaterPins[i], thermistorPins[i], 0, 0, EXRTUSION_TEMPERATURE, nullptr, 0, 0, true, false};
    pinMode(heaterPins[i], OUTPUT);
    digitalWrite(heaterPins[i], LOW); // Start with heaters off
    pinMode(heaterEnableSwitchPins[i], INPUT_PULLUP);
    heaters[i].pid = new PID(&heaters[i].input, &heaters[i].output, &heaters[i].setpoint, 38.4, 4.8, 40.7, DIRECT);
    heaters[i].pid->SetOutputLimits(0, 200);
    heaters[i].pid->SetMode(AUTOMATIC);
  }
}

#define EEPROM_PID_ADDR_BASE 0
#define EEPROM_PID_BLOCK_SIZE 12 // 3 floats (Kp, Ki, Kd) * 4 bytes

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

    // Start with heater off
    digitalWrite(heaters[heaterIndex].pin, LOW);
    delay(500);

    while (cycleCount < cycles) {
        input = getTemperature(analogRead(heaters[heaterIndex].sensorPin));

        // Print progress
        Serial.print("Tuning Heater ");
        Serial.print(heaterIndex);
        Serial.print(" | Temp: ");
        Serial.print(input, 1);
        Serial.print(" C | Output: ");
        Serial.println(output);

        // --- Thermal runaway protection ---
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

        // Heating phase
        if (heating) {
            output = power;
            digitalWrite(heaters[heaterIndex].pin, output > 0 ? HIGH : LOW);
            if (input > tuneSetpoint + 5) { // Overshoot threshold
                heating = false;
                maxTemp = input;
                t1 = millis();
            }
        } else { // Cooling phase
            output = 0;
            digitalWrite(heaters[heaterIndex].pin, LOW);
            if (input < tuneSetpoint - 5) { // Undershoot threshold
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
                // Calculate Ku and Pu for Ziegler-Nichols
                Ku = (4.0 * power) / (3.14159 * (maxTemp - minTemp));
                Pu = period / 1000.0;
            }
        }
        delay(100);
    }
    digitalWrite(heaters[heaterIndex].pin, LOW); // Turn off heater after tuning

    // Ziegler-Nichols tuning rules
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

void printMenu() {
  Serial.println("\n========================================");
  Serial.println("  PET Filament Extruder - Command Menu");
  Serial.println("========================================");
  Serial.println();
  Serial.println("HEATER COMMANDS:");
  Serial.println("  TUNE <heater> <setpoint> <power>");
  Serial.println("    - Auto-tune PID for heater (0-4)");
  Serial.println("    - Example: TUNE 0 220 200");
  Serial.println();
  Serial.println("  PID <heater>");
  Serial.println("    - Show current PID values for heater (0-4)");
  Serial.println("    - Example: PID 0");
  Serial.println();
  Serial.println("  PIDS");
  Serial.println("    - Show PID values for all heaters");
  Serial.println();
  Serial.println("  SETPID <heater> <Kp> <Ki> <Kd>");
  Serial.println("    - Manually set PID values for heater (0-4)");
  Serial.println("    - Example: SETPID 0 38.4 4.8 40.7");
  Serial.println();
  Serial.println("  STATUS");
  Serial.println("    - Show detailed status of all heaters");
  Serial.println();
  Serial.println("MOTOR COMMANDS:");
  Serial.println("  SPEED <motor> <steps/sec>");
  Serial.println("    - Set speed for motor (0-4), range 50-3000");
  Serial.println("    - Example: SPEED 0 800");
  Serial.println();
  Serial.println("  SPEEDS");
  Serial.println("    - Show all motor speeds");
  Serial.println();
  Serial.println("  MOTOR <motor>");
  Serial.println("    - Manually select motor for encoder control");
  Serial.println("    - Example: MOTOR 2");
  Serial.println();
  Serial.println("SYSTEM COMMANDS:");
  Serial.println("  HELP or MENU");
  Serial.println("    - Display this menu");
  Serial.println();
  Serial.println("  RESET");
  Serial.println("    - Reset all motor speeds to default (500)");
  Serial.println();
  Serial.println("========================================");
  Serial.println("Current Settings:");
  Serial.print("  Selected Motor: ");
  Serial.println(selectedMotorIndex);
  Serial.print("  Default Speed: ");
  Serial.print(DEFAULT_SPEED);
  Serial.println(" steps/sec");
  Serial.print("  Target Temp: ");
  Serial.print(EXRTUSION_TEMPERATURE);
  Serial.println("C");
  Serial.println("========================================\n");
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
  Serial.print("  Kp: ");
  Serial.println(Kp, 4);
  Serial.print("  Ki: ");
  Serial.println(Ki, 4);
  Serial.print("  Kd: ");
  Serial.println(Kd, 4);
}

void printAllPIDValues() {
  Serial.println("\n========== ALL PID VALUES ==========");
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    double Kp = heaters[i].pid->GetKp();
    double Ki = heaters[i].pid->GetKi();
    double Kd = heaters[i].pid->GetKd();
    
    Serial.print("Heater ");
    Serial.print(i);
    Serial.print(": Kp=");
    Serial.print(Kp, 4);
    Serial.print(" | Ki=");
    Serial.print(Ki, 4);
    Serial.print(" | Kd=");
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
  Serial.println(" PID values updated and saved to EEPROM.");
  printPIDValues(heaterIndex);
}

void printMotorSpeeds() {
  Serial.println("\nMotor Speeds:");
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    Serial.print("  Motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(steppers[i].speed);
    Serial.print(" steps/sec");
    if (i == selectedMotorIndex) {
      Serial.print(" [SELECTED]");
    }
    Serial.println();
  }
  Serial.println();
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
  Serial.println("All motor speeds reset to default (500 steps/sec)");
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
    Serial.print(" | Fault: ");
    Serial.println(thermalFault[i] ? "YES" : "NO");
  }
  Serial.println("\nMOTORS:");
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    Serial.print("  Motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(steppers[i].speed);
    Serial.print(" steps/sec | Running: ");
    Serial.print(steppers[i].enabled ? "YES" : "NO ");
    if (i == selectedMotorIndex) {
      Serial.print(" [SELECTED]");
    }
    Serial.println();
  }
  Serial.println("===================================\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Initialize heaters
  setupHeaters();
  loadAllPIDFromEEPROM();
  
  // Initialize stepper motors
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
    digitalWrite(stepperDirPins[i], HIGH);  // Set default direction
    digitalWrite(stepperEnablePins[i], HIGH); // Disable by default (active LOW)
  }
  
  // Initialize rotary encoder
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize stable LEDs for heaters
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    pinMode(heaterStableLEDPins[i], OUTPUT);
    digitalWrite(heaterStableLEDPins[i], LOW);
  }
  
  // Setup encoder interrupts
  // INT3 (pin 18) for ENCODER_CLK
  EICRA |= (1 << ISC30);  // Trigger on any logic change
  EIMSK |= (1 << INT3);   // Enable INT3
  
  // INT1 (pin 20) for ENCODER_DT
  EICRA |= (1 << ISC10);  // Trigger on any logic change
  EIMSK |= (1 << INT1);   // Enable INT1
  
  // Setup timers
  setupTimer1();  // Steppers 0-1
  setupTimer3();  // Steppers 2-3
  setupTimer5();  // Stepper 4
  setupTimer4();  // Heater control (existing)
  
  sei(); // enable interrupts
  
  Serial.println("PET Filament Extruder Controller Initialized");
  Serial.println("Software PWM enabled for all heaters (10Hz)");
  Serial.println("Use encoder to adjust speed, press button to select motor");
  Serial.print("Default speed: ");
  Serial.print(DEFAULT_SPEED);
  Serial.println(" steps/sec");
  Serial.println("\nType 'HELP' or 'MENU' for available commands\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Handle encoder button for motor selection
  uint8_t reading = digitalRead(ENCODER_SW);
  if (reading != lastButtonState) {
    lastButtonPress = millis();
  }
  
  if ((millis() - lastButtonPress) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      
      // Button pressed (LOW because of INPUT_PULLUP)
      if (buttonState == LOW && !isBlinking) {
        selectedMotorIndex = (selectedMotorIndex + 1) % NUMBER_OF_HEATERS;
        
        // Start LED blink sequence
        isBlinking = true;
        speedAdjustmentEnabled = false;
        blinkCount = selectedMotorIndex + 1;
        currentBlink = 0;
        ledState = false;
        lastBlinkTime = millis();
        digitalWrite(LED_PIN, LOW);
        
        Serial.print("Selected Motor ");
        Serial.print(selectedMotorIndex);
        Serial.print(" | Speed: ");
        Serial.print(steppers[selectedMotorIndex].speed);
        Serial.println(" steps/sec");
      }
    }
  }
  lastButtonState = reading;
  
  // LED blink state machine
  if (isBlinking) {
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = millis();
      
      if (ledState) {
        // LED is ON, turn it OFF
        digitalWrite(LED_PIN, LOW);
        ledState = false;
        currentBlink++;
        
        // Check if all blinks completed
        if (currentBlink >= blinkCount) {
          isBlinking = false;
          speedAdjustmentEnabled = true;
        }
      } else {
        // LED is OFF, turn it ON (if more blinks needed)
        if (currentBlink < blinkCount) {
          digitalWrite(LED_PIN, HIGH);
          ledState = true;
        }
      }
    }
  }
  
  // Update stepper enable states based on heater status
  for (int i = 0; i < NUMBER_OF_HEATERS; i++) {
    // Update stable LED
    if (heaters[i].stable && heaters[i].enabled) {
      digitalWrite(heaterStableLEDPins[i], HIGH);  // Turn on stable LED
      digitalWrite(stepperEnablePins[i], LOW);  // Enable stepper (active LOW)
      steppers[i].enabled = true;
    } else {
      digitalWrite(heaterStableLEDPins[i], LOW);   // Turn off stable LED
      digitalWrite(stepperEnablePins[i], HIGH); // Disable stepper
      steppers[i].enabled = false;
    }
  }
  
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
  
  // Resume status updates after 15 seconds of no serial input
  if (statusUpdatesPaused && (millis() - lastSerialInput) > STATUS_RESUME_TIMEOUT) {
    statusUpdatesPaused = false;
    Serial.println("[Status updates resumed]\n");
  }

  // Serial command handler for PID auto-tune
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    // Pause status updates and track input time
    statusUpdatesPaused = true;
    lastSerialInput = millis();
    
    if (cmd.startsWith("HELP") || cmd.startsWith("MENU")) {
      printMenu();
      
    } else if (cmd.startsWith("TUNE")) {
      // Format: TUNE <heater> <setpoint> <power>
      int idx = -1, power = 255;
      double setpoint = EXRTUSION_TEMPERATURE;
      int numArgs = sscanf(cmd.c_str(), "TUNE %d %lf %d", &idx, &setpoint, &power);
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        if (numArgs == 2) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else if (numArgs == 3) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else autoTunePID(idx, EXRTUSION_TEMPERATURE, 255, AUTOTUNE_CYCLES);
      } else {
        Serial.println("Invalid heater index. Use 0-4.");
      }
      
    } else if (cmd.startsWith("PID ")) {
      int idx = -1;
      sscanf(cmd.c_str(), "PID %d", &idx);
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        printPIDValues(idx);
      } else {
        Serial.println("Invalid heater index. Use 0-4.");
      }
      
    } else if (cmd.startsWith("PIDS")) {
      printAllPIDValues();
      
    } else if (cmd.startsWith("SETPID")) {
      // Format: SETPID <heater> <Kp> <Ki> <Kd>
      int idx = -1;
      double Kp, Ki, Kd;
      int numArgs = sscanf(cmd.c_str(), "SETPID %d %lf %lf %lf", &idx, &Kp, &Ki, &Kd);
      if (numArgs == 4 && idx >= 0 && idx < NUMBER_OF_HEATERS) {
        setPIDValues(idx, Kp, Ki, Kd);
      } else {
        Serial.println("Invalid format. Use: SETPID <heater> <Kp> <Ki> <Kd>");
      }
      
    } else if (cmd.startsWith("STATUS")) {
      printDetailedStatus();
      
    } else if (cmd.startsWith("SPEED ")) {
      // Format: SPEED <motor> <steps/sec>
      int idx = -1;
      uint16_t speed = 0;
      int numArgs = sscanf(cmd.c_str(), "SPEED %d %u", &idx, &speed);
      if (numArgs == 2 && idx >= 0 && idx < NUMBER_OF_HEATERS) {
        setMotorSpeed(idx, speed);
      } else {
        Serial.println("Invalid format. Use: SPEED <motor> <steps/sec>");
      }
      
    } else if (cmd.startsWith("SPEEDS")) {
      printMotorSpeeds();
      
    } else if (cmd.startsWith("MOTOR ")) {
      int idx = -1;
      sscanf(cmd.c_str(), "MOTOR %d", &idx);
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        selectedMotorIndex = idx;
        Serial.print("Selected Motor ");
        Serial.print(selectedMotorIndex);
        Serial.print(" | Speed: ");
        Serial.print(steppers[selectedMotorIndex].speed);
        Serial.println(" steps/sec");
      } else {
        Serial.println("Invalid motor index. Use 0-4.");
      }
      
    } else if (cmd.startsWith("RESET")) {
      resetMotorSpeeds();
      
    } else {
      Serial.println("Unknown command. Type 'HELP' for available commands.");
    }
  }
}

// put function definitions here: