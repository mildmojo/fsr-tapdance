// TODO: recovery level should be more than `fsrNoise` away from trigger level

#include "configuration.h"

#include <ADS1115_WE.h>
#include <Arduino.h>
#include <Wire.h>

#ifdef NEOPIXEL_PIN
  #include <Adafruit_NeoPixel.h>

  // Parameter 1 = number of pixels in strip
  // Parameter 2 = Arduino pin number (most are valid)
  // Parameter 3 = pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
  uint32_t stripColor = strip.Color(50, 150, 0);
#endif

const uint16_t CALIBRATION_TIME_MS = 500;
const uint16_t CALIBRATION_VALUE_COUNT = 20;
const uint16_t MAX_ADC_READING = 32768;
const uint16_t MIN_VALID_ADC_READING = 200;
const uint16_t SERIAL_REPORT_INTERVAL_MS = 100;
const int MIN_TRIGGER_COUNT = 5;
const int TRIGGER_DEBOUNCE_MS = 200;

ADS1115_WE adc = ADS1115_WE(&WIRE, ADC_I2C_ADDRESS);

uint16_t fsrFilteredValue;
uint16_t fsrTriggerLevel;
uint16_t fsrRecoveryLevel;
uint16_t fsrNoise;
uint32_t fsrRecoveryTotal;
uint32_t triggeredAt = 0;
uint32_t lastTriggerUpdateAt = 0;

bool isTriggered = false;
bool isProbeEnabled = false;
int validTriggerCount = 0;

uint32_t lastSerialReportAt = 0;
uint16_t loopCounter = 0;

void setupAdc();
void calibrateAdc();
uint16_t readAdc();
uint16_t filterAdc();
void onAdcTriggerRecover();
void setAdcTriggerWindow();
void thresholdCheck(uint16_t fsrValue);
void velocityCheck(uint16_t fsrValue);
void probeEnableCheck(uint16_t fsrValue);
void onFsrTrigger(uint16_t fsrValue);
void onFsrRecover(uint16_t fsrValue);
void updateTriggerLevel();
void serialReport(uint16_t fsrValue);

void ledOn();
void ledOff();
void outputHigh();
void outputLow();

void setup() {
  Serial.begin(115200);

  #ifdef PROBE_ENABLE_PIN
    pinMode(PROBE_ENABLE_PIN, INPUT);
    digitalWrite(PROBE_ENABLE_PIN, LOW);
  #endif

  #ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
  #endif

  #ifdef NEOPIXEL_POWER_PIN
    pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
    digitalWrite(NEOPIXEL_POWER_PIN, HIGH);
  #endif

  #ifdef NEOPIXEL_PIN
    strip.begin();
    strip.setBrightness(50);
    strip.show(); // Initialize all pixels to 'off'
  #endif

  // Turn on LED
  ledOn();
  // Output pin is HIGH when idle, LOW when triggered
  outputHigh();

  setupAdc();
  calibrateAdc();

  lastTriggerUpdateAt = millis();
  fsrFilteredValue = fsrRecoveryTotal / RECOVERY_RUNNING_AVG_COUNT;

  ledOff();
}

void loop() {
  uint16_t fsrValue = filterAdc();

  probeEnableCheck(fsrValue);

  thresholdCheck(fsrValue);
  velocityCheck(fsrValue);

  serialReport(fsrValue);

  loopCounter++;
}

void thresholdCheck(uint16_t fsrValue) {
  if (fsrValue > fsrTriggerLevel) {
    validTriggerCount = constrain(++validTriggerCount, 0, MIN_TRIGGER_COUNT);
    onFsrTrigger(fsrValue);
  }

  if (fsrValue <= fsrRecoveryLevel) {
    onFsrRecover(fsrValue);
  }
}

void velocityCheck(uint16_t fsrValue) {

}

void probeEnableCheck(uint16_t fsrValue) {
  #ifdef PROBE_ENABLE_PIN
    bool isEnableSignal = digitalRead(PROBE_ENABLE_PIN);

    // Rising edge, probe enable signaled?
    if (!isProbeEnabled && isEnableSignal) {
      // Calculate trigger thresholds based on last average.
      updateTriggerLevel();
    }

    // Update internal state.
    isProbeEnabled = isEnableSignal;

    // Probe disabled? Keep track of running average for next reset enable.
    if (!isProbeEnabled) {
      // Update basis for recovery values running average.
      fsrRecoveryTotal -= fsrRecoveryTotal / RECOVERY_RUNNING_AVG_COUNT;
      fsrRecoveryTotal += fsrValue;
    }
  #endif
}

void onFsrTrigger(uint16_t fsrValue) {
  if (isTriggered || validTriggerCount < MIN_TRIGGER_COUNT) {
    return;
  }

  // TODO: as long as we stay triggered, add timeout after which trigger level
  //       starts to decay so we eventually auto-recover.

  outputLow();
  ledOn();
  triggeredAt = millis();
  isTriggered = true;
}

void onFsrRecover(uint16_t fsrValue) {
  if (isTriggered) {
    outputHigh();
    ledOff();
    isTriggered = false;
    validTriggerCount = 0;
  }

  if (fsrValue < MIN_VALID_ADC_READING) {
    return;
  }

  // No enable signal? Recalculate trigger levels periodically in recovery, then.
  #ifndef PROBE_ENABLE_PIN
    // Update basis for recovery values running average.
    fsrRecoveryTotal -= fsrRecoveryTotal / RECOVERY_RUNNING_AVG_COUNT;
    fsrRecoveryTotal += fsrValue;

    // TODO: this keeps accumulating time in triggered mode, meaning first recovery
    //       value will always cause a trigger level update, which is bad because
    //       recovery value often rebounds below average.
    if (millis() - lastTriggerUpdateAt > TRIGGER_UPDATE_INTERVAL_MS) {
      updateTriggerLevel();
      lastTriggerUpdateAt = millis();
    }
  #endif
}

void updateTriggerLevel() {
  uint16_t fsrRecoveryAvg = fsrRecoveryTotal / RECOVERY_RUNNING_AVG_COUNT;

  #ifdef PROBE_ENABLE_PIN
    fsrTriggerLevel = fsrRecoveryAvg + 3*fsrNoise;
    fsrRecoveryLevel = fsrTriggerLevel - 1.5*fsrNoise;
  #else
    // Floor is either the configured trigger minimum or 2x the noise
    uint16_t fsrTriggerFloor = fsrRecoveryAvg + max(FSR_TRIGGER_MIN, 2*fsrNoise);
    // Ceiling is the configured trigger max
    uint16_t fsrTriggerCeiling = fsrRecoveryAvg + FSR_TRIGGER_MAX;

    fsrTriggerLevel = constrain(fsrRecoveryAvg * FSR_TRIGGER_MULTIPLIER, fsrTriggerFloor, fsrTriggerCeiling);
    fsrRecoveryLevel = fsrRecoveryAvg + (fsrTriggerLevel - fsrRecoveryAvg) * FSR_RECOVERY_MULTIPLIER;
    fsrRecoveryLevel = min(fsrRecoveryLevel, fsrTriggerLevel - 1.1*fsrNoise);
  #endif
}

void setupAdc() {
  WIRE.begin();

  if (!adc.init()) {
    Serial.println("ADS1115 not found on I2C bus! (address: " + String(ADC_I2C_ADDRESS, HEX) + ")");
  } else {
    Serial.println("ADS1115 initialized! (address: " + String(ADC_I2C_ADDRESS, HEX) + ")");
  }

  // pinMode(ADC_ALERT_PIN, INPUT_PULLUP);
  // adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setVoltageRange_mV(ADC_GAIN);
  // adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1);
  adc.setConvRate(ADS1115_860_SPS);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  // attachInterrupt(digitalPinToInterrupt(ADC_ALERT_PIN), onAdcTriggerRecover, FALLING);
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

  fsrRecoveryTotal = (uint32_t) fsrAverage * RECOVERY_RUNNING_AVG_COUNT;

  updateTriggerLevel();
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
// void onAdcTriggerRecover() {
//   fsrValue = readAdc();

//   if (fsrValue >= fsrTriggerLevel) {
//     // Triggered above trigger threshold
//     digitalWrite(LED_PIN, HIGH);
//     digitalWrite(OUTPUT_PIN, LOW);
//   } else {
//     // Recovered below recovery threshold
//     digitalWrite(LED_PIN, LOW);
//     digitalWrite(OUTPUT_PIN, HIGH);
//   }
// }

// void setAdcTriggerWindow() {
//   uint16_t now = millis();
//   if (now - lastAdcWindowUpdateAt > ADC_WINDOW_UPDATE_MS) {
//     float mvTrigger = fsrTriggerLevel / 32768.0 * 2048.0;
//     float mvRecovery = fsrRecoveryLevel / 32768.0 * 2048.0;
//     // TODO(shrapnel): replace with method that only updates limits
//     adc.setAlertModeAndLimit_V(ADS1115_WINDOW, mvTrigger / 1000.0, mvRecovery / 1000.0);
//     lastAdcWindowUpdateAt = now;
//     lastAdcWindowTrigger = fsrTriggerLevel;
//     lastAdcWindowRecovery = fsrRecoveryLevel;
//   }
// }


// Use this to adapt trigger margin to scale of average readings; smaller margin up top, bigger margin toward 0.
// float getFsrTriggerMin(float baseVal) {
//   return max(500, (MAX_ADC_READING - baseVal) / MAX_ADC_READING * fsrTriggerMin);
// }

void serialReport(uint16_t fsrValue) {
  uint32_t elapsed = millis() - lastSerialReportAt;

  if (elapsed > SERIAL_REPORT_INTERVAL_MS && Serial.available()) {
    Serial.print("loopCount:");
    Serial.print(loopCounter);

    Serial.print(",fsrNoise:");
    Serial.print(fsrNoise);

    Serial.print(",triggerLevel:");
    Serial.print(fsrTriggerLevel);

    Serial.print(",recoverLevel:");
    Serial.print(fsrRecoveryLevel);

    Serial.print(",recoveryAvg:");
    Serial.print(fsrRecoveryTotal / RECOVERY_RUNNING_AVG_COUNT);

    Serial.print(",fsrValue:");
    Serial.println(fsrValue);

    loopCounter = 0;
    lastSerialReportAt = millis();
  }
}

void ledOn() {
  digitalWrite(LED_PIN, LED_DRIVE_SIGNAL);
 
  #ifdef NEOPIXEL_PIN
    strip.setPixelColor(0, stripColor);
    strip.show();
  #endif
}

void ledOff() {
  digitalWrite(LED_PIN, !LED_DRIVE_SIGNAL);
 
  #ifdef NEOPIXEL_PIN
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
  #endif
}

void outputHigh() {
  digitalWrite(OUTPUT_PIN, HIGH);
}

void outputLow() {
  digitalWrite(OUTPUT_PIN, LOW);
}
