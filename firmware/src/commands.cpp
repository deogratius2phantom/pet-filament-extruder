#include "commands.h"
#include "heater.h"
#include "stepper.h"
#include "encoder.h"
#include "config.h"

bool statusUpdatesPaused = false;
uint32_t lastSerialInput = 0;

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
  Serial.println("    - Reset all motor speeds to default");
  Serial.println();
  Serial.println("========================================");
  Serial.println("Current Settings:");
  Serial.print("  Selected Motor: ");
  Serial.println(selectedMotorIndex);
  Serial.print("  Default Speed: ");
  Serial.print(DEFAULT_SPEED);
  Serial.println(" steps/sec");
  Serial.print("  Target Temp: ");
  Serial.print(EXTRUSION_TEMPERATURE);
  Serial.println("C");
  Serial.println("========================================\n");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    statusUpdatesPaused = true;
    lastSerialInput = millis();
    
    if (cmd.startsWith("HELP") || cmd.startsWith("MENU")) {
      printMenu();
      
    } else if (cmd.startsWith("TUNE")) {
      int idx = -1, power = 200;
      double setpoint = EXTRUSION_TEMPERATURE;
      int numArgs = sscanf(cmd.c_str(), "TUNE %d %lf %d", &idx, &setpoint, &power);
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        if (numArgs == 2) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else if (numArgs == 3) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else autoTunePID(idx, EXTRUSION_TEMPERATURE, 200, AUTOTUNE_CYCLES);
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
      printMotorStatus();
      
    } else if (cmd.startsWith("SPEED ")) {
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
