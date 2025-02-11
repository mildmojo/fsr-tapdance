#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <ADS1115_WE.h>

class Wire;
class Wire1;
class Wire2;

#ifndef DEBUG
#define DEBUG 0
#endif

#ifndef LED_PIN
#define LED_PIN LED_BUILTIN
#endif

#ifndef LED_DRIVE_SIGNAL
#define LED_DRIVE_SIGNAL HIGH
#endif

// ***Moved to platform.ini, board-specific
// #define NEOPIXEL_PIN 12
// #define NEOPIXEL_POWER_PIN 11

// ***Moved to platformio.ini, board-specific
// #define OUTPUT_PIN 20

// If defined, use this pin as an inbound signal to reset thresholds based on
// current average reading. Delete or comment out if no such pin is connected.
// HIGH to reset trigger threshold and disable running average read
// LOW to enable running average read
// ***Moved to platformio.ini, board-specific
// #define PROBE_ENABLE_PIN 22

#ifdef PROBE_ENABLE_PIN
  // The window between probe disabled and probe enabled is short, so allow
  // average to adjust faster.
  const uint16_t RECOVERY_RUNNING_AVG_COUNT = 200;
#else
  const uint16_t RECOVERY_RUNNING_AVG_COUNT = 3000;
#endif

#define ADC_I2C_ADDRESS 0x48
#define ADC_CHANNEL 0
// #define ADC_RANGE ADS1115_RANGE_4096
#define ADC_RANGE ADS1115_RANGE_2048
// #define ADC_RANGE ADS1115_RANGE_1024

// Multiplier from average reading to get trigger level
#define FSR_TRIGGER_MULTIPLIER 1.03
// Multiplier from average-to-trigger difference to get recovery level
#define FSR_RECOVERY_MULTIPLIER 0.8
// Maximum difference between average reading and trigger level
#define FSR_TRIGGER_MAX 1000
// Minimum difference between average reading and trigger level
#define FSR_TRIGGER_MIN 100

// WHEN NOT USING PROBE_ENABLE... (otherwise levels are updated on probe enable signal)
// Interval at which to update trigger and recovery levels (not updated while triggered)
#define TRIGGER_UPDATE_INTERVAL_MS 500
// Number of samples to average when determining new baseline for trigger level
#define TRIGGER_UPDATE_SAMPLE_COUNT 100

// Debounce state transitions.
#define OUTPUT_DEBOUNCE_MS 80

#ifdef ADC_I2C_BUS0
#define WIRE Wire
#endif

#ifdef ADC_I2C_BUS1
#define WIRE Wire1
#endif

#ifdef ADC_I2C_BUS2
#define WIRE Wire2
#endif

#ifndef WIRE
#define WIRE Wire
#endif


#endif
