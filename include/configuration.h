#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <ADS1115_WE.h>


#define LED_PIN LED_BUILTIN

#define OUTPUT_PIN 20
#define ADC_ALERT_PIN 21

#define ADC_I2C_ADDRESS 0x48
#define ADC_CHANNEL 0
#define ADC_GAIN ADS1115_RANGE_2048

// Multiplier from average reading to get trigger level
#define FSR_TRIGGER_MULTIPLIER 1.05
// Multiplier from average-to-trigger difference to get recovery level
#define FSR_RECOVERY_MULTIPLIER 0.8
// Maximum difference between average reading and trigger level
#define FSR_TRIGGER_MAX 500
// Minimum difference between average reading and trigger level
#define FSR_TRIGGER_MIN 100

// Interval at which to update trigger and recovery levels (only performed in recovery state)
#define TRIGGER_UPDATE_INTERVAL_MS 250
// Number of samples to average when determining new baseline for trigger level
#define TRIGGER_UPDATE_SAMPLE_COUNT 100


#endif
