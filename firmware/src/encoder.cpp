#include "encoder.h"
#include "stepper.h"
#include "config.h"
#include <pins.h>

// Encoder variables
volatile int8_t selectedMotorIndex = 0;
volatile int encoderPos = 0;
volatile uint8_t lastEncoderState = 0;
volatile uint32_t lastEncoderTime = 0;
volatile bool speedAdjustmentEnabled = true;

// Button debouncing
uint8_t lastButtonState = HIGH;
uint8_t buttonState = HIGH;
uint32_t lastButtonPress = 0;

// LED blink state machine
bool isBlinking = false;
uint8_t blinkCount = 0;
uint8_t currentBlink = 0;
bool ledState = false;
uint32_t lastBlinkTime = 0;

void setupEncoder() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Setup encoder interrupts
  EICRA |= (1 << ISC30);  // INT3 trigger on any logic change
  EIMSK |= (1 << INT3);   // Enable INT3
  
  EICRA |= (1 << ISC10);  // INT1 trigger on any logic change
  EIMSK |= (1 << INT1);   // Enable INT1
}

void handleEncoderButton() {
  uint8_t reading = digitalRead(ENCODER_SW);
  if (reading != lastButtonState) {
    lastButtonPress = millis();
  }
  
  if ((millis() - lastButtonPress) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW && !isBlinking) {
        selectedMotorIndex = (selectedMotorIndex + 1) % NUMBER_OF_HEATERS;
        
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
}

void handleLEDBlink() {
  if (isBlinking) {
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = millis();
      
      if (ledState) {
        digitalWrite(LED_PIN, LOW);
        ledState = false;
        currentBlink++;
        
        if (currentBlink >= blinkCount) {
          isBlinking = false;
          speedAdjustmentEnabled = true;
        }
      } else {
        if (currentBlink < blinkCount) {
          digitalWrite(LED_PIN, HIGH);
          ledState = true;
        }
      }
    }
  }
}
