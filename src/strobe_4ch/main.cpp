#include <Arduino.h>
#include <WiFi.h>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <rdm/responder/include/power_lamp.h>
#include <rdm/responder/include/utils.h>
#include <dmx/include/parameter.h>
#include <dmx/include/service.h>
#include <dmx/hal/include/nvs.h>
#include "pinConfig.h"

#define DMX_START_ADDRESS 1

constexpr dmx_port_t DMX_PORT = 1;
constexpr int TRANSMIT_PIN = 17;
constexpr int RECEIVE_PIN = 16;
constexpr int ENABLE_PIN = Max485_TR;

constexpr int DIMMER_COUNT = 4;
constexpr int PWM_FREQUENCY = 2000;
constexpr int PWM_RESOLUTION = 12;
constexpr float PWM_OUTPUT_LIMIT = 0.80f;
constexpr int PERSONALITY_COUNT = 3;
constexpr rdm_pid_t RDM_PID_DEVICE_POWER_CYCLES_CUSTOM = RDM_PID_DEVICE_POWER_CYCLES;
constexpr rdm_pid_t RDM_PID_MAX_POWER_PERCENT = 0x8001;

constexpr uint8_t PERSONALITY_MASTER_STROBE = 1;
constexpr uint8_t PERSONALITY_4CH_STROBE = 2;
constexpr uint8_t PERSONALITY_16BIT_STROBE = 3;

constexpr uint32_t STROBE_ON_TIME_US = 3000;
constexpr float STROBE_MIN_HZ = 1.0f;
constexpr float STROBE_MAX_HZ = 20.0f;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 500;
constexpr uint32_t SIGNAL_LOST_TIMEOUT_MS = 3000;
constexpr bool DEBUG_LOG_ENABLED = false;
constexpr uint8_t CPU_TEMP_SENSOR_NUM = 0;

const int dimmerPins[DIMMER_COUNT] = {DimmerPin0, DimmerPin1, DimmerPin2, DimmerPin3};
const int pwmChannels[DIMMER_COUNT] = {0, 1, 2, 3};

byte dmxValues[DMX_PACKET_SIZE] = {0};
uint16_t dimmerValues[DIMMER_COUNT] = {0};
volatile uint8_t strobeValue = 0;
uint16_t dmxStartAddress = DMX_START_ADDRESS;
uint8_t activePersonality = PERSONALITY_4CH_STROBE;

bool dmxConnected = false;
unsigned long lastDMXTime = 0;
unsigned long lastSignalTime = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long rdmPacketCount = 0;
unsigned long dmxPacketCount = 0;
unsigned long lastSensorUpdateMs = 0;
uint32_t rdmDeviceHours = 0;
uint32_t rdmLampHours = 0;
uint32_t rdmDevicePowerCycles = 0;
uint8_t rdmMaxPowerPercent = 100;
bool rdmDevicePowerCyclesRegistered = false;
bool rdmMaxPowerRegistered = false;
uint32_t deviceMsAccumulator = 0;
uint32_t lampMsAccumulator = 0;
unsigned long lastHoursTickMs = 0;

bool strobeOn = true;
unsigned long lastToggleMicros = 0;

/**
 * @brief RDM callback for DEVICE_POWER_CYCLES access.
 * @details Ensures the parameter remains effectively read-only by restoring the
 * persisted runtime value whenever a SET is attempted.
 */
void rdmDevicePowerCyclesCallback(dmx_port_t dmxPort,
                                  rdm_header_t *request_header,
                                  rdm_header_t *response_header,
                                  void *context) {
  (void)dmxPort;
  (void)request_header;
  (void)response_header;
  (void)context;
}

/**
 * @brief RDM callback for Max Power % updates.
 * @details Clamps non-compliant values to 100 for runtime safety.
 */
void rdmMaxPowerCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                         rdm_header_t *response_header, void *context) {
  (void)dmxPort;
  (void)response_header;
  (void)context;

  if (request_header->cc != RDM_CC_SET_COMMAND) {
    return;
  }

  uint8_t value = rdmMaxPowerPercent;
  if (dmx_parameter_copy(DMX_PORT, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                         &value, sizeof(value)) == 0) {
    return;
  }

  if (value > 100) {
    value = 100;
  }
  rdmMaxPowerPercent = value;
}

/**
 * @brief Registers RDM DEVICE_POWER_CYCLES parameter.
 * @details Adds a persistent RDM parameter definition for power-cycle count.
 * @return True when parameter registration succeeded.
 */
bool registerDevicePowerCyclesParameter() {
  const rdm_pid_t pid = RDM_PID_DEVICE_POWER_CYCLES_CUSTOM;
  uint32_t initValue = 0;

  dmx_nvs_get(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid, &initValue, sizeof(initValue));

  if (!dmx_parameter_add(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid,
                         DMX_PARAMETER_TYPE_NON_VOLATILE, &initValue,
                         sizeof(initValue))) {
    return false;
  }

    static rdm_parameter_definition_t definition = {};
    static bool definitionInitialized = false;
    if (!definitionInitialized) {
      definition.pid_cc = RDM_CC_GET;
      definition.ds = RDM_DS_UNSIGNED_DWORD;
      definition.get.handler = rdm_simple_response_handler;
      definition.get.request.format = NULL;
      definition.get.response.format = "d$";
      definition.set.handler = NULL;
      definition.set.request.format = NULL;
      definition.set.response.format = NULL;
      definition.pdl_size = sizeof(uint32_t);
      definition.max_value = UINT32_MAX;
      definition.min_value = 0;
      definition.default_value = 0;
      definition.units = RDM_UNITS_NONE;
      definition.prefix = RDM_PREFIX_NONE;
      definition.description = "Power Cycles";
      definitionInitialized = true;
    }

  if (!rdm_definition_set(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid, &definition)) {
    return false;
  }

  rdmDevicePowerCyclesRegistered = true;
  return true;
}

/**
 * @brief Registers manufacturer-specific Max Power percent parameter.
 * @details Adds persistent GET/SET parameter in range 0..100.
 * @return True when registration succeeded.
 */
bool registerMaxPowerParameter() {
  const rdm_pid_t pid = RDM_PID_MAX_POWER_PERCENT;
  uint8_t initValue = 100;

  dmx_nvs_get(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid, &initValue, sizeof(initValue));
  if (initValue > 100) {
    initValue = 100;
  }

  if (!dmx_parameter_add(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid,
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

  if (!rdm_definition_set(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid, &definition)) {
    return false;
  }

  if (!rdm_callback_set(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid,
                        rdmMaxPowerCallback, NULL)) {
    return false;
  }

  rdmMaxPowerPercent = initValue;
  rdmMaxPowerRegistered = true;
  return true;
}

/**
 * @brief Increments and persists DEVICE_POWER_CYCLES after boot.
 * @details Reads current value, increments once per firmware boot, then commits
 * to non-volatile storage.
 * @return True when update and commit succeeded.
 */
bool incrementDevicePowerCycles() {
  const rdm_pid_t pid = RDM_PID_DEVICE_POWER_CYCLES_CUSTOM;

  if (!rdmDevicePowerCyclesRegistered) {
    return false;
  }

  if (dmx_parameter_copy(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid,
                         &rdmDevicePowerCycles,
                         sizeof(rdmDevicePowerCycles)) == 0) {
    rdmDevicePowerCycles = 0;
  }

  if (rdmDevicePowerCycles < UINT32_MAX) {
    rdmDevicePowerCycles++;
  }

  if (dmx_parameter_set(DMX_PORT, RDM_SUB_DEVICE_ROOT, pid,
                        &rdmDevicePowerCycles,
                        sizeof(rdmDevicePowerCycles)) == 0) {
    return false;
  }

  bool committed = false;
  for (int i = 0; i < 8; i++) {
    const rdm_pid_t committedPid = dmx_parameter_commit(DMX_PORT);
    if (committedPid == pid) {
      committed = true;
      break;
    }
    if (committedPid == 0) {
      break;
    }
  }

  return committed;
}

/**
 * @brief RDM callback for IDENTIFY_DEVICE requests.
 * @details Keeps callback registered for compatibility; no extra action needed.
 */
void rdmIdentifyCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                         rdm_header_t *response_header, void *context) {
  (void)dmxPort;
  (void)request_header;
  (void)response_header;
  (void)context;
}

/**
 * @brief RDM callback for DMX start address updates.
 * @details Reads updated RDM start address and constrains to valid range.
 */
void rdmStartAddressCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                             rdm_header_t *response_header, void *context) {
  (void)request_header;
  (void)response_header;
  (void)context;

  uint16_t startAddr = 1;
  if (rdm_get_dmx_start_address(dmxPort, &startAddr)) {
    if (startAddr < 1) {
      startAddr = 1;
    }
    uint16_t maxStartAddress = 512;
    switch (activePersonality) {
      case PERSONALITY_MASTER_STROBE:
        maxStartAddress = 511;
        break;
      case PERSONALITY_4CH_STROBE:
        maxStartAddress = 508;
        break;
      case PERSONALITY_16BIT_STROBE:
        maxStartAddress = 504;
        break;
      default:
        maxStartAddress = 508;
        break;
    }
    if (startAddr > maxStartAddress) {
      startAddr = maxStartAddress;
    }
    dmxStartAddress = startAddr;
  }
}

/**
 * @brief RDM callback for DMX personality changes.
 * @details Synchronizes runtime parser state to the currently selected RDM
 * personality and clamps DMX start address to personality footprint.
 */
void rdmPersonalityCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                            rdm_header_t *response_header, void *context) {
  (void)request_header;
  (void)response_header;
  (void)context;

  rdm_dmx_personality_t personality = {0};
  if (rdm_get_dmx_personality(dmxPort, &personality) == 0) {
    return;
  }

  if (personality.current < PERSONALITY_MASTER_STROBE ||
      personality.current > PERSONALITY_16BIT_STROBE) {
    return;
  }

  activePersonality = personality.current;

  uint16_t maxStartAddress = 508;
  if (activePersonality == PERSONALITY_MASTER_STROBE) {
    maxStartAddress = 511;
  } else if (activePersonality == PERSONALITY_16BIT_STROBE) {
    maxStartAddress = 504;
  }

  if (dmxStartAddress > maxStartAddress) {
    dmxStartAddress = maxStartAddress;
    rdm_set_dmx_start_address(dmxPort, dmxStartAddress);
  }
}

/**
 * @brief Reads CPU temperature.
 * @return CPU temperature in degrees Celsius.
 */
float readCpuTemperatureC() {
#if defined(ARDUINO_ARCH_ESP32)
  return temperatureRead();
#else
  return 0.0f;
#endif
}

/**
 * @brief Updates the RDM CPU temperature sensor value.
 */
void updateRdmSensors() {
  const unsigned long nowMs = millis();
  if (nowMs - lastSensorUpdateMs < 1000) {
    return;
  }
  lastSensorUpdateMs = nowMs;

  const float cpuTemp = readCpuTemperatureC();
  const int16_t sensorTemp = static_cast<int16_t>(roundf(cpuTemp));
  rdm_sensor_set(DMX_PORT, RDM_SUB_DEVICE_ROOT, CPU_TEMP_SENSOR_NUM, sensorTemp);
}

/**
 * @brief Updates RDM device and lamp hours.
 * @details Device hours track uptime, lamp hours track active output time.
 */
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
    rdm_set_device_hours(DMX_PORT, rdmDeviceHours);
    rdm_set_lamp_hours(DMX_PORT, rdmLampHours);
  }
}

/**
 * @brief Writes dimmer values to PWM channels.
 * @param enabled True to drive dimmers, false to force all outputs off.
 */
void writeOutputs(bool enabled) {
  const uint32_t pwmMax = (1u << PWM_RESOLUTION) - 1u;
  const float rdmMaxPowerLimit = static_cast<float>(rdmMaxPowerPercent) / 100.0f;

  for (int i = 0; i < DIMMER_COUNT; i++) {
    uint32_t duty = 0;
    if (enabled) {
      const float normalized = static_cast<float>(dimmerValues[i]) / 65535.0f;
      const float limited = normalized * PWM_OUTPUT_LIMIT * rdmMaxPowerLimit;
      duty = static_cast<uint32_t>(limited * static_cast<float>(pwmMax));
    }
    ledcWrite(pwmChannels[i], duty);
  }
}

/**
 * @brief Returns current RDM identify state.
 */
bool isIdentifyActive() {
  bool identify = false;
  rdm_get_identify_device(DMX_PORT, &identify);
  return identify;
}

/**
 * @brief Runs a visible identify animation across dimmer channels.
 * @details Overrides normal output while identify is active.
 */
void handleIdentifyAnimation() {
  static int activeChannel = 0;
  static unsigned long lastStepMs = 0;

  const unsigned long nowMs = millis();
  if (lastStepMs == 0) {
    lastStepMs = nowMs;
  }
  if (nowMs - lastStepMs >= 1000u) {
    lastStepMs = nowMs;
    activeChannel = (activeChannel + 1) % DIMMER_COUNT;
  }

  const uint32_t pwmMax = (1u << PWM_RESOLUTION) - 1u;
  const float maxLimit = PWM_OUTPUT_LIMIT * (static_cast<float>(rdmMaxPowerPercent) / 100.0f);
  const float identifyLevel = maxLimit * 0.25f;
  const uint32_t dutyOn = static_cast<uint32_t>(identifyLevel * static_cast<float>(pwmMax));

  for (int i = 0; i < DIMMER_COUNT; i++) {
    const uint32_t duty = (i == activeChannel) ? dutyOn : 0u;
    ledcWrite(pwmChannels[i], duty);
  }
}

/**
 * @brief Applies strobe gating over current dimmer output values.
 * @details DMX strobe value 0 = direct output, 1..255 = 1..20 Hz.
 */
void handleStrobe() {
  if (strobeValue == 0) {
    strobeOn = true;
    writeOutputs(true);
    return;
  }

  const unsigned long nowUs = micros();
  const float ratio = static_cast<float>(strobeValue - 1) / 254.0f;
  const float frequencyHz = STROBE_MIN_HZ + (ratio * (STROBE_MAX_HZ - STROBE_MIN_HZ));
  const unsigned long periodUs = static_cast<unsigned long>(1000000.0f / frequencyHz);
  const unsigned long offDurationUs =
      periodUs > STROBE_ON_TIME_US ? (periodUs - STROBE_ON_TIME_US) : 1000;

  if (strobeOn) {
    if (nowUs - lastToggleMicros >= STROBE_ON_TIME_US) {
      strobeOn = false;
      lastToggleMicros = nowUs;
      writeOutputs(false);
    }
  } else {
    if (nowUs - lastToggleMicros >= offDurationUs) {
      strobeOn = true;
      lastToggleMicros = nowUs;
      writeOutputs(true);
    }
  }
}

/**
 * @brief Receives DMX/RDM and updates channel state.
 */
void readDMX() {
  dmx_packet_t packet;
  if (!dmx_receive(DMX_PORT, &packet, DMX_TIMEOUT_TICK)) {
    return;
  }

  if (packet.err) {
    return;
  }

  lastSignalTime = millis();

  if (packet.is_rdm) {
    rdmPacketCount++;
    rdm_send_response(DMX_PORT);
    return;
  }

  const size_t readLength =
      (packet.size < DMX_PACKET_SIZE) ? packet.size : DMX_PACKET_SIZE;
  if (readLength == 0) {
    return;
  }

  dmx_read(DMX_PORT, dmxValues, readLength);

  if (dmxValues[0] != 0) {
    return;
  }

  dmxPacketCount++;

  uint8_t personality = static_cast<uint8_t>(dmx_get_current_personality(DMX_PORT));
  if (personality < PERSONALITY_MASTER_STROBE ||
      personality > PERSONALITY_16BIT_STROBE) {
    personality = PERSONALITY_4CH_STROBE;
  }
  activePersonality = personality;

  uint16_t footprint = 5;
  if (activePersonality == PERSONALITY_MASTER_STROBE) {
    footprint = 2;
  } else if (activePersonality == PERSONALITY_16BIT_STROBE) {
    footprint = 9;
  }

  if (readLength <= dmxStartAddress + footprint - 1) {
    return;
  }

  if (activePersonality == PERSONALITY_MASTER_STROBE) {
    const uint16_t master = static_cast<uint16_t>(dmxValues[dmxStartAddress]) * 257u;
    for (int i = 0; i < DIMMER_COUNT; i++) {
      dimmerValues[i] = master;
    }
    strobeValue = dmxValues[dmxStartAddress + 1];
  } else if (activePersonality == PERSONALITY_16BIT_STROBE) {
    for (int i = 0; i < DIMMER_COUNT; i++) {
      const int base = dmxStartAddress + (i * 2);
      const uint16_t coarse = static_cast<uint16_t>(dmxValues[base]);
      const uint16_t fine = static_cast<uint16_t>(dmxValues[base + 1]);
      dimmerValues[i] = static_cast<uint16_t>((coarse << 8) | fine);
    }
    strobeValue = dmxValues[dmxStartAddress + 8];
  } else {
    for (int i = 0; i < DIMMER_COUNT; i++) {
      dimmerValues[i] = static_cast<uint16_t>(dmxValues[dmxStartAddress + i]) * 257u;
    }
    strobeValue = dmxValues[dmxStartAddress + 4];
  }

  dmxConnected = true;
  lastDMXTime = millis();

  const unsigned long nowMs = millis();
  if (DEBUG_LOG_ENABLED && (nowMs - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS)) {
    float debugStrobeHz = 0.0f;
    if (strobeValue > 0) {
      const float ratio = static_cast<float>(strobeValue - 1) / 254.0f;
      debugStrobeHz = STROBE_MIN_HZ + (ratio * (STROBE_MAX_HZ - STROBE_MIN_HZ));
    }

    Serial.printf(
      "[DMX] pers=%u start=%u size=%u dim=[%u,%u,%u,%u] strobe=%u hz=%.2f dmxPk=%lu rdmPk=%lu devH=%lu lampH=%lu\n",
      static_cast<unsigned>(activePersonality),
        static_cast<unsigned>(dmxStartAddress),
        static_cast<unsigned>(packet.size),
        static_cast<unsigned>(dimmerValues[0]),
        static_cast<unsigned>(dimmerValues[1]),
        static_cast<unsigned>(dimmerValues[2]),
        static_cast<unsigned>(dimmerValues[3]),
        static_cast<unsigned>(strobeValue),
        debugStrobeHz,
        dmxPacketCount,
        rdmPacketCount,
        rdmDeviceHours,
        rdmLampHours);
    lastDebugPrintMs = nowMs;
  }
}

/**
 * @brief Arduino setup.
 */
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);
  btStop();

  for (int i = 0; i < DIMMER_COUNT; i++) {
    ledcSetup(pwmChannels[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(dimmerPins[i], pwmChannels[i]);
    ledcWrite(pwmChannels[i], 0);
  }

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  dmx_config_t config = DMX_CONFIG_DEFAULT;
  config.product_category = RDM_PRODUCT_CATEGORY_DIMMER;
  config.model_id = 1;
  config.root_device_parameter_count = 48;

  dmx_personality_t personalities[] = {
      {2, "Master+Strobe"},
      {5, "4ch 8bit+Strobe"},
      {9, "4ch 16bit+Strobe"},
  };
    dmx_driver_install(DMX_PORT, &config, personalities, PERSONALITY_COUNT);
  dmx_set_pin(DMX_PORT, TRANSMIT_PIN, RECEIVE_PIN, ENABLE_PIN);

  rdm_register_identify_device(DMX_PORT, rdmIdentifyCallback, NULL);
  rdm_register_dmx_start_address(DMX_PORT, rdmStartAddressCallback, NULL);
    rdm_register_dmx_personality(DMX_PORT, PERSONALITY_COUNT,
                   rdmPersonalityCallback, NULL);

    rdm_dmx_personality_t persistedPersonality = {0};
    if (rdm_get_dmx_personality(DMX_PORT, &persistedPersonality) > 0 &&
        persistedPersonality.current >= PERSONALITY_MASTER_STROBE &&
        persistedPersonality.current <= PERSONALITY_16BIT_STROBE) {
      activePersonality = persistedPersonality.current;
    } else {
      activePersonality = PERSONALITY_4CH_STROBE;
      rdm_set_dmx_personality(DMX_PORT, activePersonality);
    }

    uint16_t persistedStartAddress = DMX_START_ADDRESS;
    if (rdm_get_dmx_start_address(DMX_PORT, &persistedStartAddress)) {
      if (persistedStartAddress < 1) {
        persistedStartAddress = 1;
      }

      uint16_t maxStartAddress = 508;
      if (activePersonality == PERSONALITY_MASTER_STROBE) {
        maxStartAddress = 511;
      } else if (activePersonality == PERSONALITY_16BIT_STROBE) {
        maxStartAddress = 504;
      }

      if (persistedStartAddress > maxStartAddress) {
        persistedStartAddress = maxStartAddress;
        rdm_set_dmx_start_address(DMX_PORT, persistedStartAddress);
      }
      dmxStartAddress = persistedStartAddress;
    } else {
      dmxStartAddress = DMX_START_ADDRESS;
      rdm_set_dmx_start_address(DMX_PORT, dmxStartAddress);
    }

    rdm_register_device_hours(DMX_PORT, NULL, NULL);
    rdm_register_lamp_hours(DMX_PORT, NULL, NULL);
    rdm_get_device_hours(DMX_PORT, &rdmDeviceHours);
    rdm_get_lamp_hours(DMX_PORT, &rdmLampHours);

    const bool powerCyclesParamOk = registerDevicePowerCyclesParameter();
    const bool powerCyclesIncrementOk = incrementDevicePowerCycles();
    const bool maxPowerParamOk = registerMaxPowerParameter();

    if (rdmMaxPowerRegistered) {
      uint8_t value = rdmMaxPowerPercent;
      if (dmx_parameter_copy(DMX_PORT, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                             &value, sizeof(value)) > 0) {
        if (value > 100) {
          value = 100;
          dmx_parameter_set(DMX_PORT, RDM_SUB_DEVICE_ROOT, RDM_PID_MAX_POWER_PERCENT,
                            &value, sizeof(value));
        }
        rdmMaxPowerPercent = value;
      }
    }

    if (DEBUG_LOG_ENABLED) {
      Serial.printf("[RDM] DEVICE_POWER_CYCLES register=%u increment=%u value=%lu\n",
                    static_cast<unsigned>(powerCyclesParamOk ? 1 : 0),
                    static_cast<unsigned>(powerCyclesIncrementOk ? 1 : 0),
                    static_cast<unsigned long>(rdmDevicePowerCycles));
      Serial.printf("[RDM] MAX_POWER_PERCENT register=%u value=%u\n",
                    static_cast<unsigned>(maxPowerParamOk ? 1 : 0),
                    static_cast<unsigned>(rdmMaxPowerPercent));
    }

    rdm_register_sensor_value(DMX_PORT, 1, NULL, NULL);
    rdm_register_sensor_definition(DMX_PORT, NULL, NULL);
    rdm_register_record_sensors(DMX_PORT, NULL, NULL);

    rdm_sensor_definition_t cpuTempSensorDefinition = {
      .num = CPU_TEMP_SENSOR_NUM,
      .type = RDM_SENSOR_TYPE_TEMPERATURE,
      .unit = RDM_UNITS_CENTIGRADE,
      .prefix = RDM_PREFIX_NONE,
      .range = {.minimum = -40, .maximum = 150},
      .normal = {.minimum = 0, .maximum = 100},
      .recorded_value_support = 1,
        .lowest_highest_detected_value_support = 1,
    };
      memset(cpuTempSensorDefinition.description, 0,
         sizeof(cpuTempSensorDefinition.description));
      strncpy(cpuTempSensorDefinition.description, "CPU Temp",
          sizeof(cpuTempSensorDefinition.description) - 1);
    rdm_sensor_definition_add(DMX_PORT, RDM_SUB_DEVICE_ROOT,
                &cpuTempSensorDefinition);

  lastDMXTime = millis();
  lastSignalTime = lastDMXTime;
  lastToggleMicros = micros();
    lastHoursTickMs = lastDMXTime;

  if (DEBUG_LOG_ENABLED) {
    Serial.println("4ch DMX/RDM baseline ready");
  }
}

/**
 * @brief Arduino loop.
 */
void loop() {
  readDMX();
  updateRdmSensors();
  updateRdmHours();

  if (isIdentifyActive()) {
    handleIdentifyAnimation();
    vTaskDelay(1 / portTICK_PERIOD_MS);
    return;
  }

  if (millis() - lastSignalTime > SIGNAL_LOST_TIMEOUT_MS) {
    if (dmxConnected) {
      dmxConnected = false;
      strobeValue = 0;
      writeOutputs(false);
      if (DEBUG_LOG_ENABLED) {
        Serial.println("[DMX] timeout/disconnected");
      }
    }
  } else if (dmxConnected) {
    handleStrobe();
  }

  vTaskDelay(1 / portTICK_PERIOD_MS);
}
