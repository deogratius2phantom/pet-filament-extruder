#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// System configuration
#define EXTRUSION_TEMPERATURE 220 // Target temperature for PET extrusion (220-235°C)
const uint8_t NUMBER_OF_HEATERS = 5;

// Timing constants
const uint32_t TIMEOUT_MS = 20000;              // Thermal runaway timeout (20 seconds)
const uint32_t AUTOTUNE_TIMEOUT_MS = 600000;    // Auto-tune timeout (10 minutes)
const int AUTOTUNE_CYCLES = 5;                  // Auto-tune cycles

// Software PWM configuration
const uint8_t PWM_PERIOD = 10;                  // 10 cycles = 1 second (10Hz)

// Stepper motor configuration
const uint16_t MIN_SPEED = 50;                  // Minimum steps/sec
const uint16_t MAX_SPEED = 3000;                // Maximum steps/sec
const uint16_t DEFAULT_SPEED = 1000;            // Default steps/sec

// Encoder configuration
const uint32_t DEBOUNCE_DELAY = 50;             // Button debounce delay (ms)
const uint32_t BLINK_INTERVAL = 200;            // LED blink interval (ms)

// Status update configuration
const uint32_t STATUS_RESUME_TIMEOUT = 15000;   // Resume status after 15 seconds

#endif // CONFIG_H
