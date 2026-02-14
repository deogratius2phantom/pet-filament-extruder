#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

extern bool statusUpdatesPaused;
extern uint32_t lastSerialInput;

void handleSerialCommands();
void printMenu();

#endif // COMMANDS_H
