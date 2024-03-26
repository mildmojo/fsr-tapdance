#include "configuration.h"

#include <ADS1115_WE.h>
#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

const uint16_t CALIBRATION_TIME_MS = 500;
const uint16_t CALIBRATION_VALUE_COUNT = 20;
const uint16_t RECOVERY_RUNNING_AVG_LENGTH = 30;
const uint16_t MAX_ADC_READING = 32768;
const uint16_t MIN_VALID_ADC_READING = 200;

const ADS1115_WE adc = ADS1115_WE(ADC_I2C_ADDRESS);

uint16_t fsrFilteredValue;
uint16_t fsrTriggerLevel;
uint16_t fsrRecoveryLevel;
uint16_t fsrNoise;
uint32_t fsrRecoveryTotal;
uint32_t lastTriggerUpdateAt = 0;

bool isTriggered = false;

void setupAdc();
void calibrateAdc();
uint16_t readAdc();
void onAdcTriggerRecover();
void setAdcTriggerWindow();
void onFsrTrigger(uint16_t fsrValue);
void onFsrRecover(uint16_t fsrValue);

void setup() {
  Serial.begin(115200);

  setupAdc();
  calibrateAdc();
}

void loop() {
  uint16_t fsrValue = filterAdc();

  if (fsrValue > fsrTriggerLevel) {
    onFsrTrigger(fsrValue);
  }

  if (fsrValue <= fsrRecoveryLevel) {
    onFsrRecover(fsrValue);
  }

  serialReport(fsrValue);
}

void onFsrTrigger(uint16_t fsrValue) {
  if (isTriggered) {
    return;
  }

  // TODO: as long as we stay triggered, add timeout after which trigger level
  //       starts to decay so we eventually auto-reset.

  digitalWrite(OUTPUT_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
  triggeredAt = micros();
  isTriggered = true;
}

void onFsrRecover(uint16_t fsrValue) {
  if (isTriggered) {
    digitalWrite(OUTPUT_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
    isTriggered = false;
  }

  if (fsrValue < MIN_VALID_ADC_READING) {
    return;
  }

  // Update basis for recovery values running average.
  fsrRecoveryTotal -= fsrRecoveryTotal / RECOVERY_RUNNING_AVG_LENGTH;
  fsrRecoveryTotal += fsrValue;

  if (abs(micros() - lastTriggerUpdateAt) > TRIGGER_UPDATE_INTERVAL_MS) {
    updateTriggerLevel(fsrValue);
    lastTriggerUpdateAt = micros();
  }
}

void updateTriggerLevel(uint16_t fsrValue) {
  uint16_t fsrRecoveryAvg = fsrRecoveryTotal / RECOVERY_RUNNING_AVG_LENGTH;

  uint16_t fsrTriggerFloor = fsrRecoveryAvg + max(FSR_TRIGGER_MIN, 2*fsrNoise);
  // Ceiling is the configured trigger max
  uint16_t fsrTriggerCeiling = fsrRecoveryAvg + FSR_TRIGGER_MAX;

  fsrTriggerLevel = constrain(fsrRecoveryAvg * FSR_TRIGGER_MULTIPLIER, fsrTriggerFloor, fsrTriggerCeiling);
  fsrRecoveryLevel = fsrRecoveryAvg + (fsrTriggerLevel - fsrRecoveryAvg) * FSR_RECOVERY_MULTIPLIER;
}

void setupAdc() {
  Wire.begin();

  if (!adc.init()) {
    Serial.println("ADS1115 not found on I2C bus! (address: " + String(ADC_I2C_ADDRESS, HEX) + ")");
  }

  // pinMode(ADC_ALERT_PIN, INPUT_PULLUP);
  adc = ADS1115_WE(ADC_I2C_ADDRESS);
  // adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setVoltageRange_mV(ADC_GAIN);
  // adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1);
  adc.setConvRate(ADS1115_860_SPS);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  attachInterrupt(digitalPinToInterrupt(ADC_ALERT_PIN), onAdcTriggerRecover, FALLING);
}

void calibrateAdc() {
  // Prime the ADC if it's been freshly configured.
  uint32_t valueTotal = 0;
  uint16_t fsrAverage;
  uint16_t fsrNoiseMin = MAX_ADC_READING - 1;
  uint16_t fsrNoiseMax = 0;
  uint16_t fsrTriggerFloor = 0;
  uint16_t fsrTriggerCeiling = 0;

  // NOLINTNEXTLINE - pretends for loop is infinite and `i` can be const. =P
  for (int i = 0; i < CALIBRATION_VALUE_COUNT; i++) {
    uint16_t reading = readAdc();

    fsrNoiseMin = min(fsrNoiseMin, reading);
    fsrNoiseMax = max(fsrNoiseMax, reading);

    valueTotal += reading;

    delay(CALIBRATION_TIME_MS / CALIBRATION_VALUE_COUNT);
  }

  fsrAverage = valueTotal / CALIBRATION_VALUE_COUNT;
  fsrNoise = fsrNoiseMax - fsrNoiseMin;
  // Floor is either the configured trigger minimum or 2x the noise
  fsrTriggerFloor = fsrAverage + max(FSR_TRIGGER_MIN, 2*fsrNoise);
  // Ceiling is the configured trigger max
  fsrTriggerCeiling = fsrAverage + FSR_TRIGGER_MAX;

  fsrTriggerLevel = constrain(fsrAverage * FSR_TRIGGER_MULTIPLIER, fsrTriggerFloor, fsrTriggerCeiling);
  fsrRecoveryLevel = fsrAverage + (fsrTriggerLevel - fsrAverage) * FSR_RECOVERY_MULTIPLIER;

  lastTriggerUpdateAt = micros();
  fsrRecoveryTotal = fsrAverage * RECOVERY_RUNNING_AVG_LENGTH;
}


uint16_t readAdc() {
  return adc.getRawResult();
}

uint16_t filterAdc() {
  uint16_t avg = (fsrFilteredValue + readAdc()) / 2;
  fsrFilteredValue = avg;
  return avg;
}

// ISR called whenever the ADC reads a value above trigger or below recovery
void onAdcTriggerRecover() {
  fsrValue = readAdc();

  if (fsrValue >= fsrTriggerLevel) {
    // Triggered above trigger threshold
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(OUTPUT_PIN, LOW);
  } else {
    // Recovered below recovery threshold
    digitalWrite(LED_PIN, LOW);
    digitalWrite(OUTPUT_PIN, HIGH);
  }
}

// void setAdcTriggerWindow() {
//   uint32_t now = micros();
//   if (now - lastAdcWindowUpdateAt > ADC_WINDOW_UPDATE_MS * 1000) {
//     float mvTrigger = fsrTriggerLevel / 32768.0 * 2048.0;
//     float mvRecovery = fsrRecoveryLevel / 32768.0 * 2048.0;
//     // TODO(shrapnel): replace with method that only updates limits
//     adc.setAlertModeAndLimit_V(ADS1115_WINDOW, mvTrigger / 1000.0, mvRecovery / 1000.0);
//     lastAdcWindowUpdateAt = now;
//     lastAdcWindowTrigger = fsrTriggerLevel;
//     lastAdcWindowRecovery = fsrRecoveryLevel;
//   }
// }


float getFsrTriggerMin(float baseVal) {
  return max(500, (MAX_ADC_READING - baseVal) / MAX_ADC_READING * fsrTriggerMin);
}

void serialReport(uint16_t fsrValue) {
  // unsigned long tick = (now - startTime)/100000;
  if (Serial.available() /*&& tick != lastOutputAt && tick % 2 == 1*/) {
    // Print number of iterations in last 200ms
    Serial.print("ct:");
    Serial.print(count);
    Serial.print(",fsrAverage:");
    Serial.print(fsrAverage);
    Serial.print(",adcTrigger:");
    Serial.print(lastAdcWindowTrigger);
    Serial.print(",adcRecovery:");
    Serial.print(lastAdcWindowRecovery);
    Serial.print(",triggerAt:");
    Serial.print(fsrTriggerLevel);
    Serial.print(",recoverAt:");
    Serial.print(fsrRecoveryLevel);
    Serial.print(",raw:");
    Serial.print(fsrValue);
    Serial.print(",filt:");
    Serial.println(filtValue);
    count = 0;
    lastOutputAt = tick;
  }
}
