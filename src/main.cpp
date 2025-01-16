#include <Arduino.h>
#include <PinConfig.h>
#include <AccelStepper.h>
#include <cstring>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <FastLED.h>
#include <Motor.h>


#define Microstepping 64
#define Acceleration 500
#define MaxSpeed 500


#define StartAddres 1
/*********ChannelList*******
1 - LED Pan
2 - LED Pan fine
3 - Ref Pan
4 - Ref Pan fine
5 - intensyty Led 1
6 - intensyty Led 2
7 - intensyty Led 3
8 - intensyty Led 4
9 - Strope
10 - LEDPixel 1 red
11 - LEDPixel 1 green
12 - LEDPixel 1 blue
13 - LEDPixel 1 white
...
***************************/

const unsigned int MaxPanSpeed = MaxSpeed;
const unsigned int MaxTiltSpeed = MaxSpeed;
const float AccelerationPan = Acceleration;
const float AccelerationTilt = Acceleration;


u_int16_t Offset_Pan = 25;
u_int16_t MaxPos_Pan = 200*14; //steps per rev*Gear reatior* 1,5 rounds


u_int16_t MaxPos_Tilt = 250; //steps per rev*Gear reatior* 0.18 rounds
u_int16_t Offset_Tilt = 5;


const int PWMfrequency = 30000;              // Set PWM frequency
const int PWMresolution = 8;                // Set PWM resolution to 8 bits
const int dimmerPins[4] = {DimmerPin0, DimmerPin1,DimmerPin2,DimmerPin3};
int ledcChannels[4] = {0, 1, 2, 3};


Motor MotorPan(M2_Dir, M2_Step, Switch_2, false, MaxPanSpeed, Acceleration, Microstepping, MaxPos_Pan, Offset_Pan);
Motor Motortilt(M1_Dir, M1_Step, Switch_1, true, MaxTiltSpeed, Acceleration, Microstepping, MaxPos_Tilt, Offset_Tilt);


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
byte  dimmerValues[4];
byte  RGBWValues[72*4];
byte strope = 0;


dmx_port_t dmxPort = 1;
uint16_t dmxStartAdresse = StartAddres;

unsigned long turnOnTime = 0;
unsigned long turnOffTime = 0;
unsigned long lastDMXTime = 0;
bool on = false;
bool enable = true;

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
byte fanValue = 0;
u_int16_t targetPos[2];
unsigned long dmxdelaymid = 30;



void dmxReadingTask(void *parameter) {
  while (true) {
    dmx_packet_t packet;
    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
      /* A packet was received! If the packet was RDM, we should send a response.
        We can do this with rdm_send_response(). If the RDM packet isn't meant for
        this device, no response will be sent. */
      if (packet.is_rdm) {
        rdm_send_response(dmxPort);
      }else {
        //rdm_get_dmx_start_address(dmxPort,*dmxStartAdresse);
        dmx_read(dmxPort, dmxValues, packet.size);
        MotorPan.setDMX(dmxValues[dmxStartAdresse],dmxValues[dmxStartAdresse+1]);
        //Serial.print("pan: ");
        //Serial.print(dmxValues[dmxStartAdresse+2]);
        //Serial.print("panfine: ");
        //Serial.println(dmxValues[dmxStartAdresse+3]);
        Motortilt.setDMX(dmxValues[dmxStartAdresse+2],dmxValues[dmxStartAdresse+3]);
      }


    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // Run this task every 50ms
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
  for(int i=0;i<4;i++){
    ledcWrite(ledcChannels[i],Value);
  }
}
TaskHandle_t ledTask;
// Function for the LED task
void ledUpdateTask(void *parameter) {
  while (true) {
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
          for(int i=0; i<4; i++){
            ledcWrite(ledcChannels[i],dimmerValues[i]);
          }
          turnOnTime = currentTime;
        }
      }
    } else {
      for(int i=0; i<4; i++){
        ledcWrite(ledcChannels[i],dimmerValues[i]);
      }

    }
   //vTaskDelay(20 / portTICK_PERIOD_MS); // Adjust this delay if needed for smoother updates
  }
}


//__________________________________Setup____________________________________________________
void setup() {
  //Fan pins
  pinMode(Fan_1, OUTPUT);
  pinMode(Fan_2, OUTPUT);
  //digitalWrite(Fan_1, HIGH);
  digitalWrite(Fan_2, LOW);
  analogWrite(Fan_1, 200);


  //Stepper Setup
  pinMode(StepperEnable, OUTPUT);
  digitalWrite(StepperEnable, LOW);

  //DMX Setup
  pinMode(Max485_TR, OUTPUT);
  digitalWrite(Max485_TR, LOW);
  delay(1000);

  //homing sequenz
  MotorPan.Rotary_homing();
  Motortilt.Switch_homing();
  


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
    {4, "PanF TiltF"}
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

  /* Care should be taken to ensure that the parameters registered for callbacks
    never go out of scope. The variables passed as parameter data for responses
    must be valid throughout the lifetime of the DMX driver. Allowing parameter
    variables to go out of scope can result in undesired behavior during RDM
    response callbacks. */

    // Create a task to handle DMX reading on core 0
    xTaskCreatePinnedToCore(
      dmxReadingTask,    // Function that implements the task
      "DMX Task",        // Name of the task
      4096 ,              // Stack size in words
      NULL,              // Task input parameter
      1,                 // Priority of the task
      &dmxTask,          // Task handle
      0);                // Run on core 0


}

void loop() {
  MotorPan.update();
  Motortilt.update();
}