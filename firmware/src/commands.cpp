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
  Serial.println("    - Auto-tune PID for heater (1-5)");
  Serial.println("    - Example: TUNE 1 220 200");
  Serial.println();
  Serial.println("  PID <heater>");
  Serial.println("    - Show current PID values for heater (1-5)");
  Serial.println("    - Example: PID 1");
  Serial.println();
  Serial.println("  PIDS");
  Serial.println("    - Show PID values for all heaters");
  Serial.println();
  Serial.println("  SETPID <heater> <Kp> <Ki> <Kd>");
  Serial.println("    - Manually set PID values for heater (1-5)");
  Serial.println("    - Example: SETPID 1 38.4 4.8 40.7");
  Serial.println();
  Serial.println("  STATUS");
  Serial.println("    - Show detailed status of all heaters");
  Serial.println();
  Serial.println("  ENABLE <heater>");
  Serial.println("    - Enable heater via software (1-5 or ALL)");
  Serial.println("    - Example: ENABLE 1 or ENABLE ALL");
  Serial.println();
  Serial.println("  DISABLE <heater>");
  Serial.println("    - Disable heater via software (1-5 or ALL)");
  Serial.println("    - Example: DISABLE 1 or DISABLE ALL");
  Serial.println();
  Serial.println("MOTOR COMMANDS:");
  Serial.println("  SPEED <motor> <steps/sec>");
  Serial.println("    - Set speed for motor (1-5), range 50-3000");
  Serial.println("    - Example: SPEED 1 800");
  Serial.println();
  Serial.println("  SPEEDS");
  Serial.println("    - Show all motor speeds");
  Serial.println();
  Serial.println("  MOTOR <motor>");
  Serial.println("    - Manually select motor for encoder control");
  Serial.println("    - Example: MOTOR 3");
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
      idx--; // Convert from 1-based to 0-based
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        if (numArgs == 2) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else if (numArgs == 3) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else autoTunePID(idx, EXTRUSION_TEMPERATURE, 200, AUTOTUNE_CYCLES);
      } else {
        Serial.println("Invalid heater index. Use 1-5.");
      }
      
    } else if (cmd.startsWith("PID ")) {
      int idx = -1;
      sscanf(cmd.c_str(), "PID %d", &idx);
      idx--; // Convert from 1-based to 0-based
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        printPIDValues(idx);
      } else {
        Serial.println("Invalid heater index. Use 1-5.");
      }
      
    } else if (cmd.startsWith("PIDS")) {
      printAllPIDValues();
      
    } else if (cmd.startsWith("SETPID")) {
      int idx = -1;
      double Kp, Ki, Kd;
      int numArgs = sscanf(cmd.c_str(), "SETPID %d %lf %lf %lf", &idx, &Kp, &Ki, &Kd);
      idx--; // Convert from 1-based to 0-based
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
      idx--; // Convert from 1-based to 0-based
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
      idx--; // Convert from 1-based to 0-based
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        selectedMotorIndex = idx;
        Serial.print("Selected Motor ");
        Serial.print(selectedMotorIndex + 1);
        Serial.print(" | Speed: ");
        Serial.print(steppers[selectedMotorIndex].speed);
        Serial.println(" steps/sec");
      } else {
        Serial.println("Invalid motor index. Use 1-5.");
      }
      
    } else if (cmd.startsWith("RESET")) {
      resetMotorSpeeds();
      
    } else if (cmd.startsWith("ENABLE")) {
      if (cmd.indexOf("ALL") != -1) {
        enableAllHeaters();
      } else {
        int idx = -1;
        sscanf(cmd.c_str(), "ENABLE %d", &idx);
        idx--; // Convert from 1-based to 0-based
        if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
          enableHeater(idx);
        } else {
          Serial.println("Invalid format. Use: ENABLE <heater> or ENABLE ALL");
        }
      }
      
    } else if (cmd.startsWith("DISABLE")) {
      if (cmd.indexOf("ALL") != -1) {
        disableAllHeaters();
      } else {
        int idx = -1;
        sscanf(cmd.c_str(), "DISABLE %d", &idx);
        idx--; // Convert from 1-based to 0-based
        if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
          disableHeater(idx);
        } else {
          Serial.println("Invalid format. Use: DISABLE <heater> or DISABLE ALL");
        }
      }
      
    } else {
      Serial.println("Unknown command. Type 'HELP' for available commands.");
    }
  }
}
