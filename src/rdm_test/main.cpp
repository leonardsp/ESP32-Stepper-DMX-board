#include <Arduino.h>
#include <WiFi.h>
#include <esp_log.h>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include "pinConfig.h"

/**
 * Minimal RDM-only responder for hardware capability verification.
 * No PWM, no FastLED, no complexity - just RDM discovery.
 */

constexpr dmx_port_t DMX_PORT = 1;
constexpr int TRANSMIT_PIN = 17;
constexpr int RECEIVE_PIN = 16;
constexpr int ENABLE_PIN = Max485_TR;
constexpr bool SERIAL_DEBUG_ENABLED = false;

const int dimmerPins[8] = {DimmerPin0, DimmerPin1, DimmerPin2, DimmerPin3, DimmerPin4, DimmerPin5, DimmerPin6, DimmerPin7};

volatile uint32_t rdmRx = 0;
volatile uint32_t rdmSent = 0;
volatile uint32_t rdmFailed = 0;
volatile uint32_t discMute = 0;
volatile uint32_t discUnMute = 0;
unsigned long lastDebugMs = 0;

// The RDM receive/respond loop runs at high FreeRTOS priority so no system
// task (esp_timer at ~22, etc.) can pre-empt it during the 2ms response window.
static void rdmHighPrioTask(void *arg) {
  (void)arg;
  while (true) {
    dmx_packet_t packet;
    if (!dmx_receive(DMX_PORT, &packet, DMX_TIMEOUT_TICK)) {
      continue;
    }
    if (packet.err) {
      continue;
    }
    if (packet.is_rdm) {
      rdmRx++;
      if (packet.size >= 23) {
        uint8_t hdr[23];
        dmx_read(DMX_PORT, hdr, 23);
        if (hdr[0] == RDM_SC && hdr[1] == RDM_SUB_SC) {
          uint16_t pid = (static_cast<uint16_t>(hdr[21]) << 8) | hdr[22];
          if (pid == RDM_PID_DISC_MUTE) discMute++;
          else if (pid == RDM_PID_DISC_UN_MUTE) discUnMute++;
        }
      }
      const bool sent = rdm_send_response(DMX_PORT);
      if (sent) rdmSent++;
      else rdmFailed++;
    }
  }
}

void rdmIdentifyNoopCallback(dmx_port_t dmxPort, rdm_header_t *request_header,
                             rdm_header_t *response_header, void *context) {
  (void)dmxPort;
  (void)request_header;
  (void)response_header;
  (void)context;
}

void setup() {
  if (SERIAL_DEBUG_ENABLED) {
    Serial.begin(115200);
  }
  delay(100);
  
  esp_log_level_set("*", ESP_LOG_NONE);
  
  if (SERIAL_DEBUG_ENABLED) {
    Serial.println("\n\nRDM-Only Test Firmware");
    Serial.println("======================");
  }
  
  // Disable WiFi/BT
  WiFi.mode(WIFI_OFF);
  btStop();
  
  // Set all dimmer pins to LOW (LEDs off)
  for (int i = 0; i < 8; i++) {
    pinMode(dimmerPins[i], OUTPUT);
    digitalWrite(dimmerPins[i], LOW);
  }
  
  // DMX pins
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  
  // Minimal DMX config
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  config.product_category = RDM_PRODUCT_CATEGORY_DIMMER;
  config.model_id = 99;
  config.root_device_parameter_count = 10;
  
  dmx_personality_t personalities[] = {
    {1, "RDM Test"}
  };
  
  dmx_driver_install(DMX_PORT, &config, personalities, 1);
  dmx_set_pin(DMX_PORT, TRANSMIT_PIN, RECEIVE_PIN, ENABLE_PIN);
  
  // Basic RDM setup
  rdm_set_device_label(DMX_PORT, "RDM Test", 8);
  rdm_register_identify_device(DMX_PORT, rdmIdentifyNoopCallback, NULL);
  rdm_register_dmx_personality(DMX_PORT, 1, NULL, NULL);

  // Pin high-priority RDM task to core 1 (same core as the UART1 ISR).
  // Priority 20 sits above the esp_timer daemon (22 is actually below configMAX_PRIORITIES-5).
  // On ESP32 Arduino configMAX_PRIORITIES=25; esp_timer runs at 22.
  // We use 19 here - above the WiFi/BT stack (disabled) but safely below watchdog.
  xTaskCreatePinnedToCore(rdmHighPrioTask, "rdm_hp", 4096, NULL, 19, NULL, 1);

  if (SERIAL_DEBUG_ENABLED) {
    Serial.println("DMX/RDM initialized");
    Serial.println("Starting discovery monitor...\n");
  }
}

void loop() {
  // All RDM work is in rdmHighPrioTask. Loop only handles debug output.
  if (!SERIAL_DEBUG_ENABLED) {
    vTaskDelay(portMAX_DELAY);
    return;
  }

  unsigned long now = millis();
  if (now - lastDebugMs >= 2000) {
    lastDebugMs = now;
    const float pct = (rdmRx > 0) ? (100.0f * rdmSent / rdmRx) : 0.0f;
    Serial.printf("[RDM] rx=%lu sent=%lu failed=%lu pct=%.1f%% mute=%lu unmute=%lu\n",
                  rdmRx, rdmSent, rdmFailed, pct, discMute, discUnMute);
  }
  vTaskDelay(pdMS_TO_TICKS(500));
}
