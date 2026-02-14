#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

extern volatile int8_t selectedMotorIndex;
extern volatile int encoderPos;
extern volatile uint8_t lastEncoderState;
extern volatile uint32_t lastEncoderTime;
extern volatile bool speedAdjustmentEnabled;

extern uint8_t lastButtonState;
extern uint8_t buttonState;
extern uint32_t lastButtonPress;

extern bool isBlinking;
extern uint8_t blinkCount;
extern uint8_t currentBlink;
extern bool ledState;
extern uint32_t lastBlinkTime;

void setupEncoder();
void handleEncoderButton();
void handleLEDBlink();

#endif // ENCODER_H
