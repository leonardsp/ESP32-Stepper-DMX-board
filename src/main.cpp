#include <Arduino.h>
#include <PinConfig.h>
#include <AccelStepper.h>
#include <cstring>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <FastLED.h>


#define Microstepping 8
#define Acceleration 5000*Microstepping
#define MaxSpeed 500*Microstepping


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

const unsigned int MaxSpeedLED = MaxSpeed*8/3;
const unsigned int MaxSpeedRef = MaxSpeed*5;
const float AccelerationLED = Acceleration*8/3;
const float AccelerationRef = Acceleration*5;


const int Offset_LED = 10*Microstepping;
const int MaxPos_LED = 200*Microstepping*8/3*2; //steps per rev*Microstepping*Gear reatior* 2 rounds
const int HomePos_LED = MaxPos_LED/2;


const int MaxPos_Ref = 200*Microstepping*5*2; //steps per rev*Microstepping*Gear reatior* 2 rounds
const int Offset_Ref = MaxPos_Ref/4;
const int HomePos_Ref = MaxPos_Ref/2;


const int PWMfrequency = 30000;              // Set PWM frequency
const int PWMresolution = 8;                // Set PWM resolution to 8 bits
const int dimmerPins[4] = {DimmerPin0, DimmerPin1,DimmerPin2,DimmerPin3};
int ledcChannels[4] = {0, 1, 2, 3};


// Create Stepper Objects
AccelStepper stepperLED(1, M1_Step, M1_Dir);
AccelStepper stepperRef(1, M2_Step, M2_Dir);

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
uint16_t PanValues[2];
byte strope = 0;


dmx_port_t dmxPort = 1;
int dmxStartAdresse = StartAddres;

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
void setStepperPosandSpeed(unsigned long dmxDel, long PanV, long TarV, AccelStepper& Stepper, unsigned int MaxSpeedStepper){
  if (dmxDel < 20) {
    dmxDel = 20; // Set a minimum delay
  }
  long stepsToMoveA = abs(PanV-TarV);
  long stepsToMoveB = abs(Stepper.distanceToGo());
  unsigned long speed = (stepsToMoveA+stepsToMoveB)*1000/dmxDel;
  if (speed < MaxSpeedStepper && speed > 100) {
    Stepper.setMaxSpeed(speed);
  } else if (speed <= 1) {
    Stepper.setMaxSpeed(1);
  } else {
    Stepper.setMaxSpeed(MaxSpeedStepper);
  }
  Stepper.moveTo(PanV);
}


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
        unsigned long now = millis(); 
        unsigned long dmxDelay = now-lastDMXTime;
        lastDMXTime = now;
        dmxdelaymid = dmxdelaymid*0.99+dmxDelay*0.01;

        dmx_read(dmxPort, dmxValues, packet.size);
        PanValues[0] = map((static_cast<uint16_t>(dmxValues[dmxStartAdresse]) << 8) | dmxValues[dmxStartAdresse+1],0,65535,0,MaxPos_LED);
        PanValues[1] = map((static_cast<uint16_t>(dmxValues[dmxStartAdresse+2]) << 8) | dmxValues[dmxStartAdresse+3],0,65535,0,MaxPos_Ref);
        // Clamp values to ensure they are within valid range
        PanValues[0] = constrain(PanValues[0], 0, MaxPos_LED);
        PanValues[1] = constrain(PanValues[1], 0, MaxPos_Ref);

        for(int i=0;i<4;i++){
          dimmerValues[i]=dmxValues[dmxStartAdresse+4+i];
        }
        strope = dmxValues[dmxStartAdresse+8];
        for(int i=0;i<288;i++){
          RGBWValues[i]=dmxValues[dmxStartAdresse+9+i];
        }

        for(int i = 0;i<2;i++){
          if(targetPos[i] != PanValues[i]){
            long stepsToMoveA = abs(PanValues[i]-targetPos[i]);
            if (i==0){
              long stepsToMoveB = abs(stepperLED.distanceToGo());
              u_int16_t speed = (stepsToMoveA+stepsToMoveB)*1000/dmxdelaymid;
              
            if (speed < MaxSpeedLED && speed > 100) {
                stepperLED.setMaxSpeed(speed);
            } else if (speed <= 100) {
                stepperLED.setMaxSpeed(100);
            } else {
                stepperLED.setMaxSpeed(MaxSpeedLED);
            }
              stepperLED.moveTo(PanValues[i]);
            }else{
              setStepperPosandSpeed(dmxdelaymid,PanValues[i],targetPos[i],stepperRef,MaxSpeedRef);
              //targetPos[i]=PanValues[i];
            }
            targetPos[i] = PanValues[i];
          }
        }
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
    vTaskDelay(20 / portTICK_PERIOD_MS); // Adjust this delay if needed for smoother updates
  }
}



bool homing(AccelStepper stepper, byte EndStopPin, int maxDis, int homepos, int offset, bool EndstopNS){
  bool b = true;
  stepper.setMaxSpeed(MaxSpeed/2);
  stepper.move(-maxDis*11/10);
  while(b) { //move while endstop is not closed (low = closed)
    stepper.run();
    
    if (EndstopNS){
      b = digitalRead(EndStopPin);  
    }else{
      b = !digitalRead(EndStopPin);
    }

    if(stepper.distanceToGo()==0){
      return false;
    }
  }
  stepper.setCurrentPosition(0);
  stepper.runToNewPosition(10);
  b = true;
  while(b) {
    stepper.move(-1);
    stepper.run();
    if (EndstopNS){
      b = digitalRead(EndStopPin);  
    }else{
      b = !digitalRead(EndStopPin);
    }
  }
  stepper.setCurrentPosition(0);
  stepper.runToNewPosition(offset);
  stepper.setCurrentPosition(0);
  delay(10);
  return true;
}



//__________________________________Setup____________________________________________________
void setup() {
  // Initialize the strip object
  FastLED.addLeds<SK6812, LEDPin, RGB>(leds, NUM_LEDS);
  //strip.begin();
  //strip.show(); // Initialize all pixels to 'off'
  //LED PWM Pins
  for (int i = 0; i < 4; i++) {
    ledcSetup(ledcChannels[i], PWMfrequency, PWMresolution);   // Setup channel with frequency and resolution
    ledcAttachPin(dimmerPins[i], ledcChannels[i]);       // Attach pin to channel
    ledcWrite(ledcChannels[i], 0);                       // Set initial duty cycle to 0 (off)
  }
  //Fan pins
  pinMode(Fan_1, OUTPUT);
  pinMode(Fan_2, OUTPUT);
  //digitalWrite(Fan_1, HIGH);
  digitalWrite(Fan_2, LOW);
  analogWrite(Fan_1, 200);
  //Endstop Pins
  pinMode(Switch_1, INPUT);
  pinMode(Switch_2, INPUT);

  //Stepper Setup
  pinMode(StepperEnable, OUTPUT);
  digitalWrite(StepperEnable, LOW);
  stepperLED.setMaxSpeed(MaxSpeedLED);
  stepperRef.setMaxSpeed(MaxSpeedRef);

  stepperLED.setAcceleration(AccelerationLED);
  stepperRef.setAcceleration(AccelerationRef);
  //DMX Setup
  pinMode(Max485_TR, OUTPUT);
  digitalWrite(Max485_TR, LOW);
  delay(1000);

  //homing sequenz
  homing(stepperRef,Switch_2,MaxPos_Ref,HomePos_Ref,Offset_Ref,true);
  if(homing(stepperLED,Switch_1,MaxPos_LED,HomePos_LED,Offset_LED,false)){
    for(int p=0;p<3;p++){
      for(int i = 0; i<4; i++){
        setAllLED(0);
        ledcWrite(ledcChannels[i],255);
        delay(100);
      }
    }
    setAllLED(0);
  }else{
    for(int i=0;i<5;i++){
      setAllLED(0);
      delay(80);
      setAllLED(255);
      delay(30);
    }
    setAllLED(0);
    digitalWrite(StepperEnable, HIGH);
    while(true){}
  }
  
    // Create a task to handle DMX reading on core 0
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
    {1, "Default Personality"}
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

  xTaskCreatePinnedToCore(
    ledUpdateTask,     // Function that implements the task
    "LED Update Task", // Name of the task
    2048,              // Stack size in words
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &ledTask,          // Task handle
    0                  // Run on core 1 (different from the DMX and stepper core)
  );


}




void loop() {
  stepperRef.run();
  stepperLED.run();

  




  /* int read = dmxRx.readPacket(DMXValue, StartAddres, 7);
  if(read == 7) {
    if(enable==false){
      digitalWrite(StepperEnable, LOW);
      enable = true;
    }
    lastDMXTime = currentTime;
    //strope
    if(DMXValue[6] != 0)
    {
      int t = (255 - DMXValue[6]) * 2;
      if(on)
      {
        const long onTime = currentTime - turnOnTime;
        if(onTime > 4)
        {
          on = false;
          //turn led off
          setAllLED(0);
          turnOffTime = currentTime;
        }
      }
      else
      {
        const long offTime = currentTime - turnOffTime;
        if(offTime > t)
        {
          on = true;
          //turn led on
          analogWrite(DimmerPin0,DMXValue[2]);
          analogWrite(DimmerPin1,DMXValue[3]);
          analogWrite(DimmerPin2,DMXValue[4]);
          analogWrite(DimmerPin3,DMXValue[5]);
          turnOnTime = currentTime;
        }
      }
    } else 
    {
        analogWrite(DimmerPin0, DMXValue[2]);
        analogWrite(DimmerPin1, DMXValue[3]);
        analogWrite(DimmerPin2, DMXValue[4]);
        analogWrite(DimmerPin3, DMXValue[5]);
    }
  
    stepperLED.setSpeed(DMXValue[0]);
    stepperRef.moveTo(map(DMXValue[1],0,255,ClosePos,OpenPos));
  } else{
    if(((currentTime-lastDMXTime)>5000)&&enable==true){ //DMX timeout
      //turn off LED's
      setAllLED(0);
      //close spheer
      stepperRef.runToNewPosition(ClosePos);
      //disable Stepper
      digitalWrite(StepperEnable, HIGH);
      enable=false;

    }
  } */
}