#include <Arduino.h>
#include "pinConfig.h"
#include <cstring>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <FastLED.h>


#define StartAddres 1
/*********ChannelList*******
5 - intensyty Led 1
6 - intensyty Led 2
7 - intensyty Led 3
8 - intensyty Led 4
9 - intensyty Led 5
10 - intensyty Led 6
11 - intensyty Led 7
12 - intensyty Led 8
13 - Strope
14 - LEDPixel 1 red
15 - LEDPixel 1 green
16 - LEDPixel 1 blue
17 - LEDPixel 1 white
...
***************************/


const int PWMfrequency = 1000;              // Set PWM frequency
const int PWMresolution = 16;                // Set PWM resolution to 16 bits
const int dimmerPins[8] = {DimmerPin0, DimmerPin1, DimmerPin2, DimmerPin3, DimmerPin4, DimmerPin5, DimmerPin6, DimmerPin7};
int ledcChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};

// LED Object
#define NUM_LEDS 72*4/3
CRGB leds[NUM_LEDS];
//uint8_t whiteChannel[NUM_LEDS];
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LEDPin, NEO_RGBW + NEO_KHZ800);



// Create the DMX receiver on Serial1.
int transmitPin = 17;
int receivePin = 16;
int enablePin = Max485_TR;
byte  dmxValues[DMX_PACKET_SIZE];
uint16_t  dimmerValues[8];
byte  RGBWValues[72*4];
byte strope = 0;


dmx_port_t dmxPort = 1;
int dmxStartAdresse = StartAddres;

unsigned long turnOnTime = 0;
unsigned long turnOffTime = 0;
unsigned long lastDMXTime = 0;
bool on = false;
bool enable = true;
bool testMode = false;

//TODO:
//int ledPin = Fan_1;
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

TaskHandle_t dmxTask;
void testDMXSend() {
  // Test sending a DMX packet
  byte testData[DMX_PACKET_SIZE];
  memset(testData, 0, DMX_PACKET_SIZE);
  testData[0] = 0; // Start code
  for(int i=1; i<10; i++) testData[i] = i * 25; // Test values
  
  dmx_write(dmxPort, testData, DMX_PACKET_SIZE);
  dmx_send(dmxPort); //, DMX_PACKET_SIZE);
  Serial.println("Sent test DMX packet");
}

void dmxReadingTask(void *parameter) {
  while (true) {
    dmx_packet_t packet;
    if (dmx_receive(dmxPort, &packet, pdMS_TO_TICKS(1000))) {
      /* A packet was received! If the packet was RDM, we should send a response.
        We can do this with rdm_send_response(). If the RDM packet isn't meant for
        this device, no response will be sent. */
      if (packet.is_rdm) {
        rdm_send_response(dmxPort);
      }else {
        memset(dmxValues, 0, DMX_PACKET_SIZE);
        dmx_read(dmxPort, dmxValues, packet.size);
        
        for(int i=0;i<8;i++){
          dimmerValues[i]=dmxValues[dmxStartAdresse+4+i] * 257;
        }
        strope = dmxValues[dmxStartAdresse+12];
        for(int i=0;i<288;i++){
          RGBWValues[i]=dmxValues[dmxStartAdresse+13+i];
        }
        // Test: Print received data for verification
        Serial.printf("DMX Packet Size: %d, First 10 values: ", packet.size);
        for(int i=0; i<10 && i<packet.size; i++) {
          Serial.printf("%d ", dmxValues[i]);
        }
        Serial.println();
        lastDMXTime = millis();
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Run this task more frequently
  }
}

// Function to set all pixels using an array of values (r, g, b, w, ...)
// Function to set RGBW values
void setAllPixels(uint8_t* colors) {
  memcpy(leds, RGBWValues, sizeof(RGBWValues));
  // Send the RGBW data to the LEDs
  FastLED.show();
}
void setAllLED(int Value){
  for(int i=0;i<8;i++){
    ledcWrite(ledcChannels[i],Value);
  }
}
TaskHandle_t ledTask;
// Function for the LED task
void ledUpdateTask(void *parameter) {
  while (true) {
    // Check for test mode
    if (millis() - lastDMXTime > 10000) {
      testMode = true;
    } else {
      testMode = false;
    }

    if (testMode) {
      // Simple sine wave animation using full 16-bit range
      for (int i = 0; i < 8; i++) {
        dimmerValues[i] = (uint16_t) round((sin(millis() / 1000.0 + i * PI / 4) + 1) * 32767.5);
      }
    }

    setAllPixels(RGBWValues);
      const unsigned long currentTime = millis();
      //strope
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
            ledcWrite(ledcChannels[i],dimmerValues[i]);
          }
          turnOnTime = currentTime;
        }
      }
    } else {
      for(int i=0; i<8; i++){
        ledcWrite(ledcChannels[i],dimmerValues[i]);
      }

    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // Adjust this delay if needed for smoother updates
  }
}



//__________________________________Setup____________________________________________________
void setup() {
  Serial.begin(115200);
  // Initialize the strip object
  FastLED.addLeds<SK6812, LEDPin, RGB>(leds, NUM_LEDS);
  //strip.begin();
  //strip.show(); // Initialize all pixels to 'off'
  //LED PWM Pins
  for (int i = 0; i < 8; i++) {
    ledcSetup(ledcChannels[i], PWMfrequency, PWMresolution);   // Setup channel with frequency and resolution
    ledcAttachPin(dimmerPins[i], ledcChannels[i]);       // Attach pin to channel
    ledcWrite(ledcChannels[i], 0);                       // Set initial duty cycle to 0 (off)
  }
  //Fan pins
  pinMode(Fan_1, OUTPUT);
  pinMode(Fan_2, OUTPUT);
  //digitalWrite(Fan_1, HIGH);
  digitalWrite(Fan_2, HIGH);
  //Fan PWM
  ledcSetup(8, PWMfrequency, PWMresolution);
  ledcAttachPin(Fan_1, 8);
  ledcWrite(8, 0);
  
  lastDMXTime = millis();  // Initialize DMX time
  xTaskCreatePinnedToCore(
    dmxReadingTask,    // Function that implements the task
    "DMX Task",        // Name of the task
    4096 ,              // Stack size in words
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &dmxTask,          // Task handle
    0);                // Run on core 0


  // Start the receiver
    /* Now we will install the DMX driver! We'll tell it which DMX port to use,
    what device configuration to use, and what DMX personalities it should have.
    If you aren't sure which configuration to use, you can use the macros
    `DMX_CONFIG_DEFAULT` to set the configuration to its default settings.
    This device is being setup as an RDM responder so it is likely that it
    should respond to DMX commands. It will need at least one DMX personality.
    Since this is an example, we will use a default personality which only uses
    1 DMX slot in its footprint. */
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {512, "Full Universe"}
  };

  int personality_count = 1;
  dmx_driver_install(dmxPort, &config, personalities, personality_count);

  /* Now set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Register the custom RDM_PID_IDENTIFY_DEVICE callback. This overwrites the
    default response. Since we aren't using a user context in the callback, we
    can pass NULL as the final argument. Don't forget to set the pin mode for
    your LED pin! */
  rdm_register_identify_device(dmxPort, rdmIdentifyCallback, NULL);

  digitalWrite(enablePin, LOW); // Ensure RS485 in receive mode

  /* Care should be taken to ensure that the parameters registered for callbacks
    never go out of scope. The variables passed as parameter data for responses
    must be valid throughout the lifetime of the DMX driver. Allowing parameter
    variables to go out of scope can result in undesired behavior during RDM
    response callbacks. */

  xTaskCreatePinnedToCore(
    ledUpdateTask,     // Function that implements the task
    "LED Update Task", // Name of the task
    2048,              // Stack size in words
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &ledTask,          // Task handle
    1                  // Run on core 1 (different from the DMX core)
  );

  // Test DMX send (remove after testing)
  // testDMXSend();

}




void loop() {
  // Main loop can be empty or handle other tasks
}