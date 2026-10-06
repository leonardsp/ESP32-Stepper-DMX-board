#include <Arduino.h>
#include "pinConfig.h"
#include <cstring>
#include <esp_dmx.h>
#include <esp_log.h>
#include <rdm/responder.h>
#include <rdm/responder/include/product_info.h>
#include <rdm/responder/include/power_lamp.h>
#include <rdm/responder/include/utils.h>
#include <dmx/include/service.h>
#include <dmx/hal/include/nvs.h>
#include <FastLED.h>
#include <WiFi.h>


#define StartAddres 249
/*********ChannelList*******
Fixture-relative channels:
8/249   - Dimmer 0 coarse
9/250   - Dimmer 0 fine
10/251   - Dimmer 1 coarse
11/252   - Dimmer 1 fine
12/253   - Dimmer 2 coarse
13/254   - Dimmer 2 fine
14/255   - Dimmer 3 coarse
15/256   - Dimmer 3 fine
16/257   - Dimmer 4 coarse
17/258  - Dimmer 4 fine
18/259  - Dimmer 5 coarse
19/260  - Dimmer 5 fine
20/261  - Dimmer 6 coarse
21/262  - Dimmer 6 fine
22/263  - Dimmer 7 coarse
24/264  - Dimmer 7 fine

25/265  - Strobe (0 = off, 1..255 = speed)

26/266  - Pixel 1 Red
27/267  - Pixel 1 Green
28/268  - Pixel 1 Blue
...
231/480 - Pixel 72 Red
232/481 - Pixel 72 Green
233/482 - Pixel 72 Blue


With StartAddres 8/249 (combined with motor_dmx at 241-247): absolute DMX channels are 249-481.
***************************/


const int PWMfrequency = 1000;              // Set PWM frequency
const int PWMresolution = 16;                // Set PWM resolution to 16 bits
const float PWM_MAX_LIMIT = 0.3;            // Limit PWM to 30% (52428 out of 65535)
const int dimmerPins[8] = {DimmerPin0, DimmerPin1, DimmerPin2, DimmerPin3, DimmerPin4, DimmerPin5, DimmerPin6, DimmerPin7};
int ledcChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};

// LED Object
#define NUM_LEDS 72
#define DIMMER_COUNT 8
#define DIMMER_CHANNELS (DIMMER_COUNT * 2)
#define STROBE_CHANNEL (DIMMER_CHANNELS + 1)
#define PIXEL_FIRST_CHANNEL (STROBE_CHANNEL + 1)
#define PIXEL_CHANNELS (NUM_LEDS * 3)
#define DMX_FOOTPRINT (PIXEL_FIRST_CHANNEL - 1 + PIXEL_CHANNELS)
#define PERSONALITY_COUNT 1
#define PERSONALITY_LED_DIMMER 1
#define DMX_MAX_START_ADDRESS (512 - DMX_FOOTPRINT + 1)
#define RDM_PID_MAX_POWER_PERCENT 0x8001
#define DEBUG_DMX_PACKET_LOG 0
#define SERIAL_DEBUG_ENABLED 0
#define LOW_POWER_IDLE_MODE 1
#define RDM_PIXEL_QUIET_WINDOW_MS 2000
#define ENFORCE_FIXED_DMX_START_ADDRESS 1

// Thermal management and telemetry tuning constants.
// Hardware-calibrated model: dimmers ~10W per channel, pixels ~0.1W per color channel.
const float DIMMER_CHANNEL_MAX_POWER_W = 10.0f;
const float PIXEL_MAX_POWER_W = 21.6f;
const float FAN_MIN_DUTY = 0.5f;
const float FAN_ENABLE_THRESHOLD = 0.18f;
const float FAN_DISABLE_THRESHOLD = 0.06f;
const bool FAN_ENABLE_ACTIVE_HIGH = true;
const unsigned long FAN_START_BOOST_MS = 1000;
const unsigned long FAN_MIN_ON_TIME_MS = 10000;
const unsigned long FAN_LOAD_RISE_TIME_MS = 45000;
const unsigned long FAN_LOAD_DECAY_TIME_MS = 180000;
const float FAN_FORCE_ON_DIMMER_THRESHOLD = 0.80f;
const float FAN_FORCE_ON_PIXEL_THRESHOLD = 0.70f;
const float ENERGY_TRACK_TEMP_C = 55.0f;
const float THERMAL_DERATE_START_C = 70.0f;
const float THERMAL_SHUTDOWN_C = 82.0f;
const unsigned long THERMAL_LOG_INTERVAL_MS = 5000;
const uint32_t LED_UPDATE_TASK_STACK_SIZE = 4096;
#define THERMAL_DEBUG_SERIAL 1

CRGB leds[NUM_LEDS];
byte RGBValues[PIXEL_CHANNELS];  // RGB: 3 bytes per LED
bool fastLedsInitialized = false;



// Create the DMX receiver on Serial1.
int transmitPin = 17;
int receivePin = 16;
int enablePin = Max485_TR;
byte  dmxValues[DMX_PACKET_SIZE];
uint16_t  dimmerValues[8];
byte strope = 0;


dmx_port_t dmxPort = 1;
uint16_t dmxStartAdresse = StartAddres;
uint8_t activePersonality = PERSONALITY_LED_DIMMER;

unsigned long turnOnTime = 0;
unsigned long turnOffTime = 0;
unsigned long lastDMXTime = 0;
unsigned long lastHoursTickMs = 0;
bool on = false;
bool enable = true;
bool testMode = false;
bool dmxConnected = false;
bool ledTaskStarted = false;
TaskHandle_t ledTask = NULL;
bool pwmInitialized = false;

float thermalOutputLimit = 1.0f;
bool thermalShutdownActive = false;
float lastEspTempC = 0.0f;
float estimatedLedPowerW = 0.0f;
float estimatedDimmerPowerW = 0.0f;
float estimatedPixelPowerW = 0.0f;
float fanLoadIntegrator = 0.0f;
bool fanWasEnabled = false;
unsigned long fanEnabledSinceMs = 0;
unsigned long fanBoostUntilMs = 0;

unsigned long lastThermalTickMs = 0;
unsigned long lastThermalLogMs = 0;
uint64_t ledOnTimeMs = 0;
uint64_t ledOnTimeAboveTempMs = 0;
double ledEnergyWh = 0.0;
double ledEnergyAboveTempWh = 0.0;

void startLedTaskIfNeeded();
void ensurePwmInitialized();
float readEspTemperatureC();
bool anyLedOutputRequested();
float estimateLedPowerW();
void updateThermalControlAndTelemetry();
bool hasFullDmxFootprint(uint16_t packetSize, uint16_t startAddress);

uint32_t rdmDeviceHours = 0;
uint32_t rdmLampHours = 0;
uint32_t deviceMsAccumulator = 0;
uint32_t lampMsAccumulator = 0;
uint8_t rdmMaxPowerPercent = 100;
bool rdmMaxPowerRegistered = false;

volatile unsigned long lastRdmPacketMs = 0;

// If DMX frames are consistently too short for this fixture footprint,
// force outputs dark after a short grace period to avoid random pixel states.
unsigned long lastShortFrameMs = 0;
const unsigned long SHORT_FRAME_FAILSAFE_MS = 500;
const uint8_t DMX_RECONNECT_IGNORE_FRAMES = 20;
uint8_t dmxReconnectFramesRemaining = 0;

// Clamps the configured DMX start address to the valid fixture range.
uint16_t clampStartAddress(uint16_t startAddress) {
  if (startAddress < StartAddres) {
    startAddress = StartAddres;
  }
  if (startAddress > DMX_MAX_START_ADDRESS) {
    startAddress = DMX_MAX_START_ADDRESS;
  }
  return startAddress;
}

// Scales a raw 16-bit dimmer value with global power and thermal limits.
uint16_t scaleDimmerValue(uint16_t value) {
  const float rdmMaxPowerLimit = static_cast<float>(rdmMaxPowerPercent) / 100.0f;
  return static_cast<uint16_t>(value * PWM_MAX_LIMIT * rdmMaxPowerLimit * thermalOutputLimit);
}

// Validates that the received DMX packet includes all slots needed by the
// configured start address and full fixture footprint.
bool hasFullDmxFootprint(uint16_t packetSize, uint16_t startAddress) {
  const uint16_t requiredLastSlot = startAddress + DMX_FOOTPRINT - 1;
  return packetSize > requiredLastSlot;
}

// Reads MCU temperature used by thermal derating logic.
float readEspTemperatureC() {
#if defined(ARDUINO_ARCH_ESP32)
  return temperatureRead();
#else
  return 25.0f;
#endif
}

// Returns true when any dimmer or pixel channel currently requests non-zero output.
bool anyLedOutputRequested() {
  for (int i = 0; i < DIMMER_COUNT; i++) {
    if (dimmerValues[i] > 0) {
      return true;
    }
  }

  for (int i = 0; i < PIXEL_CHANNELS; i++) {
    if (RGBValues[i] > 0) {
      return true;
    }
  }

  return false;
}

// Estimates LED+dimmer electrical load from current DMX values and limits.
float estimateLedPowerW() {
  const float rdmMaxPowerLimit = static_cast<float>(rdmMaxPowerPercent) / 100.0f;

  // Strobe reduces average dimmer power: duty = onTime / (onTime + offTime).
  // onTime is fixed at 4 ms; offTime = (255 - strope) * 2 ms (strobe logic in LED task).
  float strobeDutyCycle = 1.0f;
  if (strope != 0) {
    const float onTime = 4.0f;
    const float offTime = static_cast<float>((255 - strope) * 2);
    strobeDutyCycle = onTime / (onTime + offTime);
  }

  float dimmerRatioSum = 0.0f;
  for (int i = 0; i < DIMMER_COUNT; i++) {
    dimmerRatioSum += static_cast<float>(dimmerValues[i]) / 65535.0f;
  }
  estimatedDimmerPowerW = dimmerRatioSum * DIMMER_CHANNEL_MAX_POWER_W * rdmMaxPowerLimit * strobeDutyCycle;

  uint32_t pixelLevelSum = 0;
  for (int i = 0; i < PIXEL_CHANNELS; i++) {
    pixelLevelSum += RGBValues[i];
  }
  const float pixelRatio = static_cast<float>(pixelLevelSum) /
                           static_cast<float>(PIXEL_CHANNELS * 255.0f);
  estimatedPixelPowerW = pixelRatio * PIXEL_MAX_POWER_W * rdmMaxPowerLimit;

  return estimatedDimmerPowerW + estimatedPixelPowerW;
}

// Updates thermal derating, fan control, and cumulative runtime energy telemetry.
void updateThermalControlAndTelemetry() {
  const unsigned long nowMs = millis();
  if (lastThermalTickMs == 0) {
    lastThermalTickMs = nowMs;
    lastThermalLogMs = nowMs;
  }

  const unsigned long deltaMs = nowMs - lastThermalTickMs;
  lastThermalTickMs = nowMs;

  lastEspTempC = readEspTemperatureC();
  estimatedLedPowerW = estimateLedPowerW();

  if (lastEspTempC <= THERMAL_DERATE_START_C) {
    thermalOutputLimit = 1.0f;
    thermalShutdownActive = false;
  } else if (lastEspTempC >= THERMAL_SHUTDOWN_C) {
    thermalOutputLimit = 0.0f;
    thermalShutdownActive = true;
  } else {
    const float tempSpan = THERMAL_SHUTDOWN_C - THERMAL_DERATE_START_C;
    thermalOutputLimit = (THERMAL_SHUTDOWN_C - lastEspTempC) / tempSpan;
    thermalOutputLimit = constrain(thermalOutputLimit, 0.0f, 1.0f);
    thermalShutdownActive = false;
  }

  const bool ledRequestedOn = anyLedOutputRequested();
  if (ledRequestedOn) {
    ledOnTimeMs += deltaMs;
  }

  const float effectivePowerW = estimatedLedPowerW * thermalOutputLimit;
  ledEnergyWh += static_cast<double>(effectivePowerW) *
                 static_cast<double>(deltaMs) / 3600000.0;

  if (lastEspTempC >= ENERGY_TRACK_TEMP_C) {
    if (ledRequestedOn) {
      ledOnTimeAboveTempMs += deltaMs;
    }
    ledEnergyAboveTempWh += static_cast<double>(effectivePowerW) *
                            static_cast<double>(deltaMs) / 3600000.0;
  }

  const float modelMaxPowerW = (DIMMER_COUNT * DIMMER_CHANNEL_MAX_POWER_W) + PIXEL_MAX_POWER_W;
  const float normalizedLoad = constrain(estimatedLedPowerW / modelMaxPowerW, 0.0f, 1.0f);

  float maxDimmerRatio = 0.0f;
  for (int i = 0; i < DIMMER_COUNT; i++) {
    const float ratio = static_cast<float>(dimmerValues[i]) / 65535.0f;
    if (ratio > maxDimmerRatio) {
      maxDimmerRatio = ratio;
    }
  }

  uint32_t pixelLevelSum = 0;
  for (int i = 0; i < PIXEL_CHANNELS; i++) {
    pixelLevelSum += RGBValues[i];
  }
  const float pixelRatio = static_cast<float>(pixelLevelSum) /
                           static_cast<float>(PIXEL_CHANNELS * 255.0f);
  const bool forceFanOn =
      (maxDimmerRatio >= FAN_FORCE_ON_DIMMER_THRESHOLD) ||
      (pixelRatio >= FAN_FORCE_ON_PIXEL_THRESHOLD);

  const bool loadIncreasing = normalizedLoad > fanLoadIntegrator;
  const unsigned long fanLoadTimeMs = loadIncreasing
      ? FAN_LOAD_RISE_TIME_MS
      : FAN_LOAD_DECAY_TIME_MS;
  const float fanLoadStep = constrain(
      static_cast<float>(deltaMs) / static_cast<float>(fanLoadTimeMs), 0.0f, 1.0f);

  fanLoadIntegrator += (normalizedLoad - fanLoadIntegrator) * fanLoadStep;
  fanLoadIntegrator = constrain(fanLoadIntegrator, 0.0f, 1.0f);

  float fanDemandRaw = fanLoadIntegrator;
  float fanDemand = fanDemandRaw;

  bool fanEnabled = fanWasEnabled;
  if (fanWasEnabled) {
    if (fanLoadIntegrator <= FAN_DISABLE_THRESHOLD &&
        nowMs - fanEnabledSinceMs >= FAN_MIN_ON_TIME_MS) {
      fanEnabled = false;
    }
  } else if (fanLoadIntegrator >= FAN_ENABLE_THRESHOLD) {
    fanEnabled = true;
  }

  if (forceFanOn) {
    fanEnabled = true;
    if (fanDemand < FAN_MIN_DUTY) {
      fanDemand = FAN_MIN_DUTY;
    }
  }

  if (!fanEnabled) {
    fanDemand = 0.0f;
  } else if (fanDemand < FAN_MIN_DUTY) {
    fanDemand = FAN_MIN_DUTY;
  }

  if (fanEnabled && !fanWasEnabled) {
    fanEnabledSinceMs = nowMs;
    fanBoostUntilMs = nowMs + FAN_START_BOOST_MS;
  } else if (!fanEnabled) {
    fanBoostUntilMs = 0;
  }

  float fanDrive = fanDemand;
  const bool fanBoostActive = fanEnabled && nowMs < fanBoostUntilMs;
  if (fanBoostActive) {
    fanDrive = 1.0f;
  }

  const uint16_t fanPwm = static_cast<uint16_t>(fanDrive * 65535.0f);
  if (pwmInitialized) {
    digitalWrite(Fan_1, (fanEnabled == FAN_ENABLE_ACTIVE_HIGH) ? HIGH : LOW);
    ledcWrite(8, fanPwm);
  }
  fanWasEnabled = fanEnabled;

  if ((nowMs - lastThermalLogMs) >= THERMAL_LOG_INTERVAL_MS) {
#if SERIAL_DEBUG_ENABLED
    const float ledOnSeconds = static_cast<float>(ledOnTimeMs) / 1000.0f;
    const float ledOnAboveTempSeconds = static_cast<float>(ledOnTimeAboveTempMs) / 1000.0f;
#if THERMAL_DEBUG_SERIAL
    const char *thermalState = "NORMAL";
    if (thermalShutdownActive) {
      thermalState = "SHUTDOWN";
    } else if (thermalOutputLimit < 1.0f) {
      thermalState = "DERATE";
    }

    Serial.print("ThermalDbg: T=");
    Serial.print(lastEspTempC, 1);
    Serial.print("C state=");
    Serial.print(thermalState);
    Serial.print(" out=");
    Serial.print(thermalOutputLimit * 100.0f, 1);
    Serial.print("% fanEn=");
    Serial.print(fanEnabled ? 1 : 0);
    Serial.print(" fanBoost=");
    Serial.print(fanBoostActive ? 1 : 0);
    Serial.print(" fanInt=");
    Serial.print(fanLoadIntegrator * 100.0f, 1);
    Serial.print("% fanRaw=");
    Serial.print(fanDemandRaw * 100.0f, 1);
    Serial.print("% fan=");
    Serial.print(fanDemand * 100.0f, 1);
    Serial.print("% fanDrv=");
    Serial.print(fanDrive * 100.0f, 1);
    Serial.print("% pwrRaw=");
    Serial.print(estimatedLedPowerW, 2);
    Serial.print("W(dim=");
    Serial.print(estimatedDimmerPowerW, 2);
    Serial.print(",pix=");
    Serial.print(estimatedPixelPowerW, 2);
    Serial.print(") pwrEff=");
    Serial.print(effectivePowerW, 2);
    Serial.print("W ledOn=");
    Serial.print(static_cast<unsigned int>(ledOnSeconds));
    Serial.print(" aboveT=");
    Serial.print(static_cast<unsigned int>(ledOnAboveTempSeconds));
    Serial.print(" E=");
    Serial.print(ledEnergyWh, 3);
    Serial.print("Wh E_aboveT=");
    Serial.print(ledEnergyAboveTempWh, 3);
    Serial.println("Wh");
#else
    Serial.printf("Thermal: %.1fC, fan %.0f%%, out_limit %.0f%%, power %.1fW, on %.0fs, aboveT %.0fs, E %.3fWh, E_aboveT %.3fWh\n",
                  lastEspTempC,
                  fanDemand * 100.0f,
                  thermalOutputLimit * 100.0f,
                  effectivePowerW,
                  ledOnSeconds,
                  ledOnAboveTempSeconds,
                  ledEnergyWh,
                  ledEnergyAboveTempWh);
#endif
#endif
    lastThermalLogMs = nowMs;
  }
}

// Handles SET commands for custom max-power parameter and clamps to 0..100%.
void rdmMaxPowerCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                         rdm_header_t *response_header, void *context) {
  (void)dmxPort;
  (void)response_header;
  (void)context;

  if (request_header->cc != RDM_CC_SET_COMMAND) {
    return;
  }

  uint8_t value = rdmMaxPowerPercent;
  if (dmx_parameter_copy(dmxPort, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                         &value, sizeof(value)) == 0) {
    return;
  }

  if (value > 100) {
    value = 100;
    dmx_parameter_set(dmxPort, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                      &value, sizeof(value));
  }

  rdmMaxPowerPercent = value;
}

// Registers the custom RDM max-power parameter and its callback handlers.
bool registerMaxPowerParameter(dmx_port_t dmxPort) {
  const rdm_pid_t pid = RDM_PID_MAX_POWER_PERCENT;
  uint8_t initValue = 100;

  dmx_nvs_get(dmxPort, RDM_SUB_DEVICE_ROOT, pid, &initValue, sizeof(initValue));
  if (initValue > 100) {
    initValue = 100;
  }

  if (!dmx_parameter_add(dmxPort, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &initValue,
                         sizeof(initValue))) {
    return false;
  }

  static rdm_parameter_definition_t definition = {};
  static bool definitionInitialized = false;
  if (!definitionInitialized) {
    definition.pid_cc = RDM_CC_GET_SET;
    definition.ds = RDM_DS_UNSIGNED_BYTE;
    definition.get.handler = rdm_simple_response_handler;
    definition.get.request.format = NULL;
    definition.get.response.format = "b$";
    definition.set.handler = rdm_simple_response_handler;
    definition.set.request.format = "b$";
    definition.set.response.format = NULL;
    definition.pdl_size = sizeof(uint8_t);
    definition.max_value = 100;
    definition.min_value = 0;
    definition.default_value = 100;
    definition.units = RDM_UNITS_NONE;
    definition.prefix = RDM_PREFIX_NONE;
    definition.description = "Max Power %";
    definitionInitialized = true;
  }

  if (!rdm_definition_set(dmxPort, RDM_SUB_DEVICE_ROOT, pid, &definition)) {
    return false;
  }

  if (!rdm_callback_set(dmxPort, RDM_SUB_DEVICE_ROOT, pid,
                        rdmMaxPowerCallback, NULL)) {
    return false;
  }

  rdmMaxPowerPercent = initValue;
  rdmMaxPowerRegistered = true;
  return true;
}

// Accumulates and persists device/lamp runtime hours for RDM telemetry.
void updateRdmHours() {
  const unsigned long nowMs = millis();
  if (lastHoursTickMs == 0) {
    lastHoursTickMs = nowMs;
    return;
  }

  const uint32_t deltaMs = static_cast<uint32_t>(nowMs - lastHoursTickMs);
  lastHoursTickMs = nowMs;
  deviceMsAccumulator += deltaMs;

  bool lampOn = false;
  for (int i = 0; i < DIMMER_COUNT; i++) {
    if (dimmerValues[i] > 0) {
      lampOn = true;
      break;
    }
  }

  if (lampOn && dmxConnected) {
    lampMsAccumulator += deltaMs;
  }

  bool writeHours = false;
  while (deviceMsAccumulator >= 3600000u) {
    deviceMsAccumulator -= 3600000u;
    rdmDeviceHours++;
    writeHours = true;
  }

  while (lampMsAccumulator >= 3600000u) {
    lampMsAccumulator -= 3600000u;
    rdmLampHours++;
    writeHours = true;
  }

  if (writeHours) {
    rdm_set_device_hours(dmxPort, rdmDeviceHours);
    rdm_set_lamp_hours(dmxPort, rdmLampHours);
  }
}

//TODO:
//int ledPin = Fan_1;
// Responds to RDM identify requests (placeholder hook for identify indicator output).
void rdmIdentifyCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                         rdm_header_t *response_header, void *context) {
  /* We should only turn the LED on and off when we send a SET response message.
    This prevents extra work from being done when a GET request is received. */
  if (request_header->cc == RDM_CC_SET_COMMAND) {
    bool identify;
    rdm_get_identify_device(dmxPort, &identify);
    //digitalWrite(ledPin, identify);
  }
}

// Keeps RDM DMX start address synchronized and clamped to supported bounds.
void rdmStartAddressCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                             rdm_header_t *response_header, void *context) {
  (void)request_header;
  (void)response_header;
  (void)context;

  uint16_t startAddr = StartAddres;
  if (rdm_get_dmx_start_address(dmxPort, &startAddr)) {
    const uint16_t clampedStartAddr = clampStartAddress(startAddr);
    if (clampedStartAddr != startAddr) {
      rdm_set_dmx_start_address(dmxPort, clampedStartAddr);
    }
    dmxStartAdresse = clampedStartAddr;
  }
}

// Enforces the single supported personality and validates addressing after changes.
void rdmPersonalityCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                            rdm_header_t *response_header, void *context) {
  (void)request_header;
  (void)response_header;
  (void)context;

  rdm_dmx_personality_t personality = {0};
  if (rdm_get_dmx_personality(dmxPort, &personality) == 0) {
    return;
  }

  if (personality.current != PERSONALITY_LED_DIMMER) {
    activePersonality = PERSONALITY_LED_DIMMER;
    rdm_set_dmx_personality(dmxPort, PERSONALITY_LED_DIMMER);
  } else {
    activePersonality = personality.current;
  }

  const uint16_t clampedStartAddr = clampStartAddress(dmxStartAdresse);
  if (clampedStartAddr != dmxStartAdresse) {
    dmxStartAdresse = clampedStartAddr;
    rdm_set_dmx_start_address(dmxPort, dmxStartAdresse);
  }
}

// Receives one DMX/RDM packet and updates fixture state from active channel mapping.
void readDmxOnce() {
  dmx_packet_t packet;
  if (!dmx_receive_num(dmxPort, &packet,
                       dmxStartAdresse + DMX_FOOTPRINT, DMX_TIMEOUT_TICK)) {
    return;
  }

  if (packet.err) {
    return;
  }

  if (packet.is_rdm) {
    lastDMXTime = millis();
    lastRdmPacketMs = lastDMXTime;
    rdm_send_response(dmxPort);
    return;
  }

  if (packet.sc != DMX_SC) {
    return;
  }

  ensurePwmInitialized();

  memset(dmxValues, 0, DMX_PACKET_SIZE);
  dmx_read(dmxPort, dmxValues, packet.size);

  const int dmxBaseIndex = dmxStartAdresse;
  if (!hasFullDmxFootprint(packet.size, dmxStartAdresse)) {
    const unsigned long nowMs = millis();
    if (lastShortFrameMs == 0) {
      lastShortFrameMs = nowMs;
    }

    // Hold previous output briefly; if short frames persist, blackout safely.
    if ((nowMs - lastShortFrameMs) >= SHORT_FRAME_FAILSAFE_MS) {
      dmxConnected = false;
      strope = 0;
      for (int i = 0; i < DIMMER_COUNT; i++) {
        dimmerValues[i] = 0;
      }
      memset(RGBValues, 0, sizeof(RGBValues));
    }
    return;
  }

  lastShortFrameMs = 0;

  // Require several consecutive complete frames before applying new values.
  if (!dmxConnected && dmxReconnectFramesRemaining == 0) {
    dmxReconnectFramesRemaining = DMX_RECONNECT_IGNORE_FRAMES;
    dmxConnected = true;
    lastDMXTime = millis();
    startLedTaskIfNeeded();
  }
  if (dmxReconnectFramesRemaining > 0) {
    dmxReconnectFramesRemaining--;
    lastDMXTime = millis();
    strope = 0;
    memset(dimmerValues, 0, sizeof(dimmerValues));
    memset(RGBValues, 0, sizeof(RGBValues));
    return;
  }

  for(int i = 0; i < DIMMER_COUNT; i++){
    const int coarseIndex = dmxBaseIndex + (2 * i);
    const int fineIndex = coarseIndex + 1;
    dimmerValues[i] = (dmxValues[coarseIndex] << 8) | dmxValues[fineIndex];
  }

  strope = dmxValues[dmxBaseIndex + (STROBE_CHANNEL - 1)];

  for(int i = 0; i < PIXEL_CHANNELS; i++){
    RGBValues[i] = dmxValues[dmxBaseIndex + (PIXEL_FIRST_CHANNEL - 1) + i];
  }
#if SERIAL_DEBUG_ENABLED && DEBUG_DMX_PACKET_LOG
  {
    Serial.printf("DMX Packet Size: %d, First 10 values: ", packet.size);
    for(int i=0; i<10 && i<packet.size; i++) {
      Serial.printf("%d ", dmxValues[i]);
    }
    Serial.println();
  }
#endif

  lastDMXTime = millis();
  dmxConnected = true;
  startLedTaskIfNeeded();
}

// Initializes FastLED once before any pixel buffer writes.
void ensureFastLedsInitialized() {
  if (!fastLedsInitialized) {
    FastLED.addLeds<WS2813, LEDPin, RGB>(leds, NUM_LEDS);
    fastLedsInitialized = true;
  }
}

// Copies RGB byte data into the LED buffer and flushes it to the strip.
void setAllPixels(const uint8_t *colors) {
  ensureFastLedsInitialized();
  memcpy(leds, colors, sizeof(RGBValues));
  FastLED.setBrightness(static_cast<uint8_t>(255.0f * thermalOutputLimit));
  // Send the RGB data to the LEDs
  FastLED.show();
}

// Generates idle animation content when no DMX stream is present.
void updateIdlePixelAnimation() {
  const uint8_t hueBase = static_cast<uint8_t>(millis() / 18);
  const uint8_t wavePhase = static_cast<uint8_t>(millis() / 6);

  for (int i = 0; i < NUM_LEDS; i++) {
    const uint8_t hue = hueBase + (i * 4) + (sin8(wavePhase + (i * 5)) >> 4);
    const uint8_t brightness = qadd8(70, scale8(sin8(wavePhase + (i * 11)), 140));

    CRGB pixelColor = CHSV(hue, 220, brightness);

    RGBValues[(i * 3)] = pixelColor.r;
    RGBValues[(i * 3) + 1] = pixelColor.g;
    RGBValues[(i * 3) + 2] = pixelColor.b;
  }
}

// Sets all dimmer PWM channels to the same 16-bit value.
void setAllLED(int Value){
  if (!pwmInitialized) {
    return;
  }
  uint16_t scaledValue = scaleDimmerValue(Value);
  for(int i=0;i<8;i++){
    ledcWrite(ledcChannels[i], scaledValue);
  }
}

// Configures dimmer and fan PWM channels once.
void ensurePwmInitialized() {
  if (pwmInitialized) {
    return;
  }

  for (int i = 0; i < 8; i++) {
    ledcSetup(ledcChannels[i], PWMfrequency, PWMresolution);
    ledcAttachPin(dimmerPins[i], ledcChannels[i]);
    ledcWrite(ledcChannels[i], 0);
  }

  ledcSetup(8, PWMfrequency, PWMresolution);
  ledcAttachPin(Fan_1, 8);
  ledcWrite(8, 0);

  pwmInitialized = true;
}
void ledUpdateTask(void *parameter);

// Starts the LED update FreeRTOS task the first time valid DMX is observed.
void startLedTaskIfNeeded() {
  if (ledTaskStarted) {
    return;
  }

  xTaskCreatePinnedToCore(
    ledUpdateTask,
    "LED Update Task",
    LED_UPDATE_TASK_STACK_SIZE,
    NULL,
    1,
    &ledTask,
    0
  );
  ledTaskStarted = true;
}

// Main rendering/control task for dimmers, pixels, thermal states, and strobe timing.
void ledUpdateTask(void *parameter) {
  (void)parameter;
  while (true) {
    // Enter idle test mode if DMX times out.
    if (millis() - lastDMXTime > 10000) {
      testMode = true;
      dmxConnected = false;
    } else {
      testMode = false;
    }

    updateRdmHours();
    updateThermalControlAndTelemetry();

    // During RDM-only discovery there is no DMX data to render. Keep outputs
    // off and avoid LED bus activity so responder timing stays clean.
    if (!dmxConnected && !testMode) {
      strope = 0;
      setAllLED(0);
      FastLED.clear(true);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }

    if (thermalShutdownActive) {
      strope = 0;
      setAllLED(0);
      FastLED.clear(true);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }

    if (testMode) {
#if LOW_POWER_IDLE_MODE
      for (int i = 0; i < 8; i++) {
        dimmerValues[i] = 0;
      }
      memset(RGBValues, 0, sizeof(RGBValues));
      strope = 0;
#else
      // Simple sine wave animation using full 16-bit range
      for (int i = 0; i < 8; i++) {
        dimmerValues[i] = (uint16_t) round((sin(millis() / 1000.0 + i * PI / 4) + 1) * 32767.5);
      }

      updateIdlePixelAnimation();
#endif
    }

    const unsigned long currentTime = millis();
    const bool rdmQuietActive = (currentTime - lastRdmPacketMs) < RDM_PIXEL_QUIET_WINDOW_MS;
    if (rdmQuietActive) {
      setAllLED(0);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    setAllPixels(RGBValues);

    // Strobe dimmer channels with fixed 4 ms on-time and variable off-time.
    if(strope != 0){
      int t = (255 - strope) * 2;
      if(on){
        const long onTime = currentTime - turnOnTime;
        if(onTime > 4){
          on = false;
          //turn led off
          setAllLED(0);
          turnOffTime = currentTime;
        }
      } else {
        const long offTime = currentTime - turnOffTime;
        if(offTime > t) {
          on = true;
          //turn led on
          for(int i=0; i<8; i++){
            uint16_t scaledValue = scaleDimmerValue(dimmerValues[i]);
            ledcWrite(ledcChannels[i], scaledValue);
          }
          turnOnTime = currentTime;
        }
      }
    } else {
      for(int i=0; i<8; i++){
        uint16_t scaledValue = scaleDimmerValue(dimmerValues[i]);
        ledcWrite(ledcChannels[i], scaledValue);
      }

    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // Adjust this delay if needed for smoother updates
  }
}



//__________________________________Setup____________________________________________________
// Initializes hardware, DMX/RDM stack, and persisted runtime configuration.
void setup() {
#if SERIAL_DEBUG_ENABLED
  Serial.begin(115200);
#endif
  esp_log_level_set("*", ESP_LOG_NONE);

  // Disable WiFi and Bluetooth to save power and reduce heat
  WiFi.mode(WIFI_OFF);
  btStop();
  //Fan pins
  pinMode(Fan_1, OUTPUT);
  digitalWrite(Fan_1, LOW);
  // Keep fan driver disabled at boot. It is enabled dynamically in the thermal loop.
  digitalWrite(Fan_1, FAN_ENABLE_ACTIVE_HIGH ? LOW : HIGH);

  // RS485 transceiver direction pin
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  
  lastDMXTime = millis();  // Initialize DMX time
  // Start the receiver
    /* Now we will install the DMX driver! We'll tell it which DMX port to use,
    what device configuration to use, and what DMX personalities it should have.
    If you aren't sure which configuration to use, you can use the macros
    `DMX_CONFIG_DEFAULT` to set the configuration to its default settings.
    This device is being setup as an RDM responder so it is likely that it
    should respond to DMX commands. It will need at least one DMX personality.
    Since this is an example, we will use a default personality which only uses
    1 DMX slot in its footprint. */
  // Configure DMX with RDM support
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  config.model_id = 2;  // Model ID for LED/Dimmer fixture
  config.product_category = RDM_PRODUCT_CATEGORY_DIMMER;
  config.root_device_parameter_count = 48;
  
  dmx_personality_t personalities[] = {
    {DMX_FOOTPRINT, "8ch dimmer + strobe + 72px RGB"}
  };

  int personality_count = 1;
    // Drive all LED-related outputs low before any PWM or FastLED setup so the
    // fixture stays dark until valid DMX arrives.
    for (int i = 0; i < DIMMER_COUNT; i++) {
      pinMode(dimmerPins[i], OUTPUT);
      digitalWrite(dimmerPins[i], LOW);
    }
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);
  dmx_driver_install(dmxPort, &config, personalities, personality_count);

  /* Now set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  // Set device label (already registered by driver, just update it)
  rdm_set_device_label(dmxPort, "LED Dimmer", 10);
  
  rdm_register_dmx_personality(dmxPort, PERSONALITY_COUNT,
                   rdmPersonalityCallback, NULL);

  rdm_dmx_personality_t persistedPersonality = {0};
  if (rdm_get_dmx_personality(dmxPort, &persistedPersonality) > 0 &&
      persistedPersonality.current == PERSONALITY_LED_DIMMER) {
    activePersonality = persistedPersonality.current;
  } else {
    activePersonality = PERSONALITY_LED_DIMMER;
    rdm_set_dmx_personality(dmxPort, activePersonality);
  }

  uint16_t persistedStartAddress = StartAddres;
#if ENFORCE_FIXED_DMX_START_ADDRESS
  dmxStartAdresse = clampStartAddress(StartAddres);
  rdm_set_dmx_start_address(dmxPort, dmxStartAdresse);
#else
  if (rdm_get_dmx_start_address(dmxPort, &persistedStartAddress)) {
    persistedStartAddress = clampStartAddress(persistedStartAddress);
    dmxStartAdresse = persistedStartAddress;
    rdm_set_dmx_start_address(dmxPort, dmxStartAdresse);
  } else {
    dmxStartAdresse = clampStartAddress(StartAddres);
    rdm_set_dmx_start_address(dmxPort, dmxStartAdresse);
  }
#endif

  /* Register the custom RDM_PID_IDENTIFY_DEVICE callback. This overwrites the
    default response. Since we aren't using a user context in the callback, we
    can pass NULL as the final argument. Don't forget to set the pin mode for
    your LED pin! */
  rdm_register_identify_device(dmxPort, rdmIdentifyCallback, NULL);
#if !ENFORCE_FIXED_DMX_START_ADDRESS
  rdm_register_dmx_start_address(dmxPort, rdmStartAddressCallback, NULL);
#endif
  rdm_register_device_hours(dmxPort, NULL, NULL);
  rdm_register_lamp_hours(dmxPort, NULL, NULL);
  rdm_get_device_hours(dmxPort, &rdmDeviceHours);
  rdm_get_lamp_hours(dmxPort, &rdmLampHours);

  const bool maxPowerParamOk = registerMaxPowerParameter(dmxPort);
  if (maxPowerParamOk) {
    uint8_t value = rdmMaxPowerPercent;
    if (dmx_parameter_copy(dmxPort, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                           &value, sizeof(value)) > 0) {
      if (value > 100) {
        value = 100;
        dmx_parameter_set(dmxPort, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                          &value, sizeof(value));
      }
      rdmMaxPowerPercent = value;
    }
  }
  
  /* Care should be taken to ensure that the parameters registered for callbacks
    never go out of scope. The variables passed as parameter data for responses
    must be valid throughout the lifetime of the DMX driver. Allowing parameter
    variables to go out of scope can result in undesired behavior during RDM
    response callbacks. */

  lastHoursTickMs = millis();
}




// Services DMX input continuously while the update task handles rendering.
void loop() {
  readDmxOnce();
  vTaskDelay(1 / portTICK_PERIOD_MS);
}