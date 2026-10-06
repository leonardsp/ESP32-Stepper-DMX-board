#include <Arduino.h>
#include "pinConfig.h"
#include "FastAccelStepper.h"
#include <cstring>
#include <math.h>
#include <esp_dmx.h>
#include <rdm/responder.h>
#include <rdm/responder/include/product_info.h>
#include <WiFi.h>

#define Microstepping 32
#define Acceleration 1500*Microstepping
#define MaxSpeed 300*Microstepping //500*Microstepping im ersten Prototyp

// Diagnostic mode: bypass DMX/homing entirely and just oscillate the Ref axis.
// Enable temporarily to verify Ref motor movement in isolation (scope/observe GPIO18/19).
#define TEST_REF_MOTOR_ONLY 0

/*********ChannelList*******
Absolute channels (start address 241):
1/241 - LED Pan
2/242 - LED Pan fine
3/243 - Ref Pan
4/244 - Ref Pan fine
5/245 - LED Pan Inv (0 Stop / 1-127 ccw / 128 Stop /129-255 cw)
6/246 - Ref Pan Inv (0 Stop / 1-127 ccw / 128 Stop /129-255 cw)
7/247 - Reset (0 No 1-255 Start Homing)
8-248/249-481 led_dimmer main.cpp for LED and Dimmer channels
***************************/
uint16_t dmxStartAdresse  = 1; // Default DMX start address 1 or 241 to fit 2 in one universe

const int GerarRatio_LED = 3; // 20 teeth / 60 teeth
const int GerarRatio_Ref = 4; // 20 teeth / 80 teeth
const unsigned int MaxSpeedLED = MaxSpeed*GerarRatio_LED;
const unsigned int MaxSpeedRef = MaxSpeed*GerarRatio_Ref;
const int AccelerationLED = Acceleration*GerarRatio_LED;  // FastAccelStepper nutzt Hz/s
const int AccelerationRef = Acceleration*GerarRatio_Ref;  // FastAccelStepper nutzt Hz/s


const int Offset_LED = (46-50)*GerarRatio_LED*Microstepping; //lamp 1: 46, lamp 2 has +90° offset
const int MaxPos_LED = 200*Microstepping*GerarRatio_LED*2; //steps per rev*Microstepping*Gear reatior* 2 rounds
const int HomePos_LED = MaxPos_LED/2;

const int Offset_Ref = 26*GerarRatio_Ref*Microstepping; //lamp 1: 16, lamp 2 has +90° offset
const int MaxPos_Ref = 200*Microstepping*GerarRatio_Ref*2; //steps per rev*Microstepping*Gear reatior* 2 rounds
const int HomePos_Ref = MaxPos_Ref/2;

// Create FastAccelStepper Engine and Stepper Objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperLED = NULL;
FastAccelStepper *stepperRef = NULL;

// DMX receiver using esp_dmx (DMX only, no RDM)
int transmitPin = 17;
int receivePin = 16;
int enablePin = Max485_TR;
byte  dmxValues[DMX_PACKET_SIZE] = {0}; // Initialize to zeros
volatile uint16_t PanValues[2] = {0, 0}; // Initialize to zeros

dmx_port_t dmxPort = 1;

unsigned long lastDMXTime = 0;
bool enable = true;

// DMX timeout and idle mode
const unsigned long DMX_TIMEOUT_MS = 20 * 1000; // 60 seconds in milliseconds
volatile bool motorsIdle = false; // True when motors are disabled due to DMX timeout
volatile bool needsRehoming = false; // True when DMX returns after timeout

// Startup flag to prevent position task from interfering with homing
volatile bool setupComplete = false;
volatile bool isHoming = false; // Flag set during homing to pause position task

// RDM device hours tracking
unsigned long deviceStartMillis = 0;
unsigned long lastDeviceHoursUpdate = 0;

// Temperature tracking
float temperatureMin = 999.0;  // Initialize to impossible high value
float temperatureMax = -999.0; // Initialize to impossible low value
bool temperatureInitialized = false;
bool homingSingleAxis(FastAccelStepper* stepper, byte hallSensorPin, int maxDis, int offset, unsigned int axisMaxSpeed, int axisAcceleration) {
  // Home eine einzelne Achse mit einem Hall-Sensor
  const char* axisName = (hallSensorPin == Switch_1) ? "LED" : "Ref";
  
  Serial.print("Homing ");
  Serial.print(axisName);
  Serial.print(" - Initial sensor state: ");
  Serial.println(digitalRead(hallSensorPin));
  
  // Setze Homing-Speed und Acceleration (langsamer für Zuverlässigkeit)
  stepper->setSpeedInHz(axisMaxSpeed/4);  // War /2, jetzt /4
  stepper->setAcceleration(axisAcceleration/4);  // War /2, jetzt /4
  
  // Wenn Hall-Sensor bereits getriggert ist, bewege zuerst weg
  if (!digitalRead(hallSensorPin)) { // Sensor ist LOW (getriggert)
    Serial.println("  Sensor already triggered, moving away...");
    stepper->move(maxDis/16); // Bewege 1/16 der maximalen Distanz vorwärts
    while(stepper->isRunning()) { delay(1); }
    delay(100); // Kurze Pause
    Serial.print("  After move away, sensor: ");
    Serial.println(digitalRead(hallSensorPin));
  }
  
  // Bewege in negative Richtung bis Hall-Sensor getriggert wird
  Serial.println("  Searching for sensor...");
  stepper->move(-maxDis/2);
  delay(200);  // Längere Wartezeit damit Bewegung sicher gestartet ist
  
  // Verify stepper is actually moving
  if (!stepper->isRunning()) {
    Serial.println("  WARNING: Stepper not running after move command!");
    delay(100);
  }
  
  Serial.print("  Motor running: ");
  Serial.print(stepper->isRunning());
  Serial.print(", Target: ");
  Serial.println(stepper->targetPos());
  
  // Warte bis Hall-Sensor getriggert wird oder Bewegung stoppt
  unsigned long searchStart = millis();
  while(digitalRead(hallSensorPin)) { // Warte bis LOW (Sensor getriggert)
    if(!stepper->isRunning()) {
      Serial.print("  ERROR: Sensor not found! Time: ");
      Serial.print(millis() - searchStart);
      Serial.print("ms, Final pos: ");
      Serial.print(stepper->getCurrentPosition());
      Serial.print(", Sensor: ");
      Serial.println(digitalRead(hallSensorPin));
      stepper->setSpeedInHz(axisMaxSpeed);
      stepper->setAcceleration(axisAcceleration);
      return false;
    }
    delay(1);
  }
  
  Serial.println("  Sensor found!");
  
  // Sensor gefunden, stoppe und setze temporäre Position
  stepper->forceStopAndNewPosition(0);
  delay(10);
  
  // Bewege etwas weg vom Sensor
  stepper->moveTo(200);
  while(stepper->isRunning()) { delay(1); }
  
  // Langsam zurück für präzise Position (1/15 Geschwindigkeit - guter Kompromiss)
  stepper->setSpeedInHz(axisMaxSpeed/15);
  stepper->setAcceleration(axisAcceleration/15);
  stepper->move(-400);
  
  // Warte bis Sensor wieder getriggert wird
  while(digitalRead(hallSensorPin)) { // Warte bis LOW
    if (!stepper->isRunning()) {
      stepper->move(-1);
    }
    delay(1);
  }
  
  // Präzise Position gefunden:
  // 1) physisch zu Offset fahren
  // 2) logische Position auf Center setzen
  stepper->forceStopAndNewPosition(0);
  stepper->setSpeedInHz(axisMaxSpeed);
  stepper->setAcceleration(axisAcceleration);
  stepper->moveTo(offset);
  while(stepper->isRunning()) { delay(1); }
  int centerPos = maxDis / 2;
  stepper->setCurrentPosition(centerPos);
  
  Serial.print("  ");
  Serial.print(axisName);
  Serial.println(" homing complete");
  delay(10);
  return true;
}

bool homeBothAxes() {
  // Komplette Homing-Sequenz für beide Achsen mit separaten Hall-Sensoren
  // Gibt true zurück wenn erfolgreich, false bei Fehler
  Serial.println("Homing...");
  isHoming = true;
  
  // Home LED-Achse mit Switch_1
  if (!homingSingleAxis(stepperLED, Switch_1, MaxPos_LED, Offset_LED, MaxSpeedLED, AccelerationLED)) {
    isHoming = false;
    return false; // LED-Achse Homing fehlgeschlagen
  }
  
  // Home Ref-Achse mit Switch_2
  if (!homingSingleAxis(stepperRef, Switch_2, MaxPos_Ref, Offset_Ref, MaxSpeedRef, AccelerationRef)) {
    isHoming = false;
    return false; // Ref-Achse Homing fehlgeschlagen
  }
  
  Serial.println("Homing complete\n");
  isHoming = false;
  return true; // Homing erfolgreich
}

void testHomingAccuracy(int iterations = 5) {
  Serial.print("\nTesting homing accuracy (");
  Serial.print(iterations);
  Serial.println(" cycles)...\n");
  
  long ledPositions[10];
  long refPositions[10];
  
  // Test LED axis
  for (int i = 0; i < iterations && i < 10; i++) {
    // Move away from sensor first
    if (i > 0) {
      stepperLED->move(5000);
      while(stepperLED->isRunning()) { delay(1); }
      delay(100);
    }
    
    // Slow approach to sensor (1/15 speed - same as homing)
    stepperLED->setSpeedInHz(MaxSpeedLED/15);
    stepperLED->setAcceleration(AccelerationLED/15);
    stepperLED->move(-MaxPos_LED);
    delay(50);
    
    // Wait for sensor trigger
    while(digitalRead(Switch_1)) {
      if(!stepperLED->isRunning()) {
        Serial.println("ERROR: LED sensor not found!");
        return;
      }
      delay(1);
    }
    
    // Record position at trigger (trigger edge approach)
    stepperLED->forceStop();
    delay(10);
    ledPositions[i] = stepperLED->getCurrentPosition();
    
    // Reset speed
    stepperLED->setSpeedInHz(MaxSpeedLED);
    stepperLED->setAcceleration(AccelerationLED);
  }
  
  // Calculate LED statistics
  long ledMin = ledPositions[0];
  long ledMax = ledPositions[0];
  long ledSum = 0;
  for (int i = 0; i < iterations; i++) {
    ledSum += ledPositions[i];
    if (ledPositions[i] < ledMin) ledMin = ledPositions[i];
    if (ledPositions[i] > ledMax) ledMax = ledPositions[i];
  }
  long ledAvg = ledSum / iterations;
  long ledRange = ledMax - ledMin;
  
  // Calculate degrees (steps per rev = 200 * Microstepping * GerarRatio_LED)
  float stepsPerRevLED = 200.0 * Microstepping * GerarRatio_LED;
  float ledRangeDegrees = (ledRange / stepsPerRevLED) * 360.0;
  
  Serial.println("LED Axis:");
  Serial.print("  Range: ");
  Serial.print(ledRange);
  Serial.print(" microsteps = ");
  Serial.print((float)ledRange / Microstepping, 2);
  Serial.print(" full steps = ");
  Serial.print(ledRangeDegrees, 4);
  Serial.println(" degrees");
  
  // Test Ref axis
  for (int i = 0; i < iterations && i < 10; i++) {
    // Move away from sensor first
    if (i > 0) {
      stepperRef->move(5000);
      while(stepperRef->isRunning()) { delay(1); }
      delay(100);
    }
    
    // Slow approach to sensor (1/15 speed - same as homing)
    stepperRef->setSpeedInHz(MaxSpeedRef/15);
    stepperRef->setAcceleration(AccelerationRef/15);
    stepperRef->move(-MaxPos_Ref);
    delay(50);
    
    // Wait for sensor trigger
    while(digitalRead(Switch_2)) {
      if(!stepperRef->isRunning()) {
        Serial.println("ERROR: Ref sensor not found!");
        return;
      }
      delay(1);
    }
    
    // Record position at trigger (trigger edge approach)
    stepperRef->forceStop();
    delay(10);
    refPositions[i] = stepperRef->getCurrentPosition();
    
    // Reset speed
    stepperRef->setSpeedInHz(MaxSpeedRef);
    stepperRef->setAcceleration(AccelerationRef);
  }
  
  // Calculate Ref statistics
  long refMin = refPositions[0];
  long refMax = refPositions[0];
  long refSum = 0;
  for (int i = 0; i < iterations; i++) {
    refSum += refPositions[i];
    if (refPositions[i] < refMin) refMin = refPositions[i];
    if (refPositions[i] > refMax) refMax = refPositions[i];
  }
  long refAvg = refSum / iterations;
  long refRange = refMax - refMin;
  
  // Calculate degrees (steps per rev = 200 * Microstepping * GerarRatio_Ref)
  float stepsPerRevRef = 200.0 * Microstepping * GerarRatio_Ref;
  float refRangeDegrees = (refRange / stepsPerRevRef) * 360.0;
  
  Serial.println("Ref Axis:");
  Serial.print("  Range: ");
  Serial.print(refRange);
  Serial.print(" microsteps = ");
  Serial.print((float)refRange / Microstepping, 2);
  Serial.print(" full steps = ");
  Serial.print(refRangeDegrees, 4);
  Serial.println(" degrees\n");
}

TaskHandle_t dmxTask;
TaskHandle_t positionTask;

byte fanValue = 0;
long targetPos[2] = {0, 0}; // Initialize to zeros
double rotationOffset[2] = {0.0, 0.0}; // Accumulated position from rotation (double to avoid per-loop truncation drift)
byte rotationSpeed[2] = {0, 0}; // Initialize to stopped (0 = stop)
unsigned long lastPositionUpdateTime = 0;

// DMX frame interpolation for smooth low-speed movement
volatile long previousPanValues[2] = {0, 0};
volatile long currentPanValues[2] = {0, 0};
volatile unsigned long lastDMXReceiveTime = 0;
const unsigned long DMX_FRAME_INTERVAL = 23; // ~44Hz DMX refresh rate
volatile unsigned long dmxInterpolationInterval = DMX_FRAME_INTERVAL;
portMUX_TYPE motionDataMux = portMUX_INITIALIZER_UNLOCKED; // Protects access to motion data (previous/current pan values, rotation speed, last DMX receive time, interpolation interval)

double smoothedCommandPos[2] = {0.0, 0.0};
double smoothedCommandVel[2] = {0.0, 0.0};
bool smoothingInitialized = false;

const float POSITION_TRACK_KP = 18.0f;  // Higher gain to recover long-move responsiveness
const float COMMAND_ACCEL_FACTOR = 4.0f; // Command trajectory accel > motor accel to avoid double filtering
const float HOLD_VELOCITY_EPS = 5.0f;   // Steps/s threshold for considering axis settled
const float HOLD_POSITION_EPS = 1.0f;   // Steps threshold for snap-to-target at rest

inline bool isRotationHoldValue(byte speed) {
  return speed == 128;
}

void updateAxisTracking(FastAccelStepper* stepper, int axisIndex, long desiredPos, uint32_t axisMaxSpeed, int32_t axisAcceleration, float dtSeconds) {
  float positionError = (float)desiredPos - (float)smoothedCommandPos[axisIndex];
  float desiredVelocity = positionError * POSITION_TRACK_KP;
  desiredVelocity = constrain(desiredVelocity, -(float)axisMaxSpeed, (float)axisMaxSpeed);

  float velocityError = desiredVelocity - (float)smoothedCommandVel[axisIndex];
  float maxVelocityDelta = (float)axisAcceleration * COMMAND_ACCEL_FACTOR * dtSeconds;
  velocityError = constrain(velocityError, -maxVelocityDelta, maxVelocityDelta);
  smoothedCommandVel[axisIndex] += velocityError;
  smoothedCommandPos[axisIndex] += smoothedCommandVel[axisIndex] * dtSeconds;

  if (fabsf(positionError) < HOLD_POSITION_EPS && fabsf((float)smoothedCommandVel[axisIndex]) < HOLD_VELOCITY_EPS) {
    smoothedCommandPos[axisIndex] = (double)desiredPos;
    smoothedCommandVel[axisIndex] = 0.0;
  }

  long smoothedTarget = (long)roundf((float)smoothedCommandPos[axisIndex]);
  if (targetPos[axisIndex] != smoothedTarget) {
    stepper->moveTo(smoothedTarget);
    targetPos[axisIndex] = smoothedTarget;
  }
}

void positionCalculationTask(void *parameter) {
  // Handles position updates based on:
  // - Pan values (absolute position)
  // - Rotation speed (infinite rotation with position tracking)
  // - Combined final position sent to steppers
  
  // Wait for setup to complete before starting
  while (!setupComplete) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  // Initialize timing after setup completes
  lastPositionUpdateTime = millis();
  lastDMXReceiveTime = millis();
  
  byte prevRotationSpeed[2] = {0, 0}; // Track previous speed to detect changes
  
  while (true) {
    unsigned long now = millis();
    unsigned long timeSinceLastUpdate = now - lastPositionUpdateTime;
    lastPositionUpdateTime = now;

    long previousPanSnapshot[2];
    long currentPanSnapshot[2];
    byte rotationSpeedSnapshot[2];
    unsigned long lastDMXReceiveSnapshot = 0;
    unsigned long interpolationIntervalSnapshot = DMX_FRAME_INTERVAL;
    portENTER_CRITICAL(&motionDataMux);
    previousPanSnapshot[0] = previousPanValues[0];
    previousPanSnapshot[1] = previousPanValues[1];
    currentPanSnapshot[0] = currentPanValues[0];
    currentPanSnapshot[1] = currentPanValues[1];
    rotationSpeedSnapshot[0] = rotationSpeed[0];
    rotationSpeedSnapshot[1] = rotationSpeed[1];
    lastDMXReceiveSnapshot = lastDMXReceiveTime;
    interpolationIntervalSnapshot = dmxInterpolationInterval;
    portEXIT_CRITICAL(&motionDataMux);
    
    // Skip position updates if motors are idle or homing is in progress
    if (motorsIdle || isHoming) {
      smoothingInitialized = false;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    
    if (!smoothingInitialized) {
      smoothedCommandPos[0] = (double)stepperLED->getCurrentPosition();
      smoothedCommandPos[1] = (double)stepperRef->getCurrentPosition();
      smoothedCommandVel[0] = 0.0;
      smoothedCommandVel[1] = 0.0;
      smoothingInitialized = true;
    }

    float dtSeconds = max(0.001f, (float)timeSinceLastUpdate / 1000.0f);

    // Interpolate between DMX frames using the measured frame interval.
    long interpolatedPanValues[2];
    unsigned long timeSinceDMX = now - lastDMXReceiveSnapshot;
    
    for (int i = 0; i < 2; i++) {
      unsigned long interpolationInterval = interpolationIntervalSnapshot;
      if (interpolationInterval == 0) {
        interpolatedPanValues[i] = currentPanSnapshot[i];
        continue;
      }

      float t = min(1.0f, (float)timeSinceDMX / (float)interpolationInterval);
      long delta = currentPanSnapshot[i] - previousPanSnapshot[i];
      interpolatedPanValues[i] = previousPanSnapshot[i] + (long)roundf((float)delta * t);
    }
    
    long commandedPanValues[2] = {interpolatedPanValues[0], interpolatedPanValues[1]};

    // Calculate position for each axis (LED and Ref)
    for (int i = 0; i < 2; i++) {
      byte speed = rotationSpeedSnapshot[i];
      
      // Rotation mode mapping:
      // 0   => idle: return to nearest equivalent pan position (shortest path)
      // 128 => stop: hold current offset
      // else => continuous rotation
      if (speed == 0) {
        // Select the equivalent turn from the received pan value, not the
        // interpolated previous frame. This limits the final return move to
        // half a mechanical turn even when pan and rotation stop change together.
        if (prevRotationSpeed[i] != 0) {
          long currentPos = (i == 0) ? stepperLED->getCurrentPosition() : stepperRef->getCurrentPosition();
          long repeatPeriod = (i == 0) ? MaxPos_LED / 2 : MaxPos_Ref / 2;
          long panBase = currentPanSnapshot[i];
          long cycles = round((float)(currentPos - panBase) / repeatPeriod);
          rotationOffset[i] = (double)(cycles * repeatPeriod);
          commandedPanValues[i] = panBase;
        }
      } else if (!isRotationHoldValue(speed)) {
        // Speed range: 1-127 (ccw), 129-255 (cw)
        // Calculate rotation rate proportional to distance from center (128)
        int rotationRate;
        long cMaxSpeed = (i == 0) ? MaxSpeedLED : MaxSpeedRef;
        if (speed < 128) {
          // CCW rotation: 1 = slowest, 127 = fastest
          rotationRate = (speed - 1) * cMaxSpeed / 127;
          rotationOffset[i] += rotationRate * timeSinceLastUpdate / 1000.0; // Add to offset
        } else {
          // CW rotation: 129 = slowest, 255 = fastest
          rotationRate = (speed - 129) * cMaxSpeed / 127;
          rotationOffset[i] -= rotationRate * timeSinceLastUpdate / 1000.0; // Subtract from offset
        }
        
        // Overflow protection: re-home if rotationOffset exceeds safe limit
        if (fabs(rotationOffset[i]) > 2000000000.0) {
          // Stop rotation on both axes before homing
          portENTER_CRITICAL(&motionDataMux);
          rotationSpeed[0] = 0;
          rotationSpeed[1] = 0;
          portEXIT_CRITICAL(&motionDataMux);
          prevRotationSpeed[0] = 0;
          prevRotationSpeed[1] = 0;
          
          // Re-home both axes
          bool homingSuccess = homeBothAxes();
          
          if (homingSuccess) {
            rotationOffset[0] = 0;
            rotationOffset[1] = 0;
          }
          // TODO: User-Feedback wenn Homing nach Overflow fehlschlägt
        }
      }
      
      prevRotationSpeed[i] = speed; // Update previous speed for next iteration
    }
    
    // Calculate final target position = pan value + rotation offset (continuous, no wrap)
    long calculatedPos[2];
    calculatedPos[0] = commandedPanValues[0] + (long)rotationOffset[0];
    calculatedPos[1] = commandedPanValues[1] + (long)rotationOffset[1];
    
    updateAxisTracking(stepperLED, 0, calculatedPos[0], MaxSpeedLED, AccelerationLED, dtSeconds);
    updateAxisTracking(stepperRef, 1, calculatedPos[1], MaxSpeedRef, AccelerationRef, dtSeconds);
    
    vTaskDelay(5 / portTICK_PERIOD_MS); // Run every 5ms for smoother interpolation
  }
}


// Simple DMX reading task
void dmxReadingTask(void *parameter) {
  while (true) {
    dmx_packet_t packet;
    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) {
      if (packet.err) {
        taskYIELD();
        continue;
      }

      if (packet.is_rdm) {
        // Handle RDM requests - respond to them
        rdm_send_response(dmxPort);
      } else {
        if (packet.sc != DMX_SC) {
          taskYIELD();
          continue;
        }

        // Process DMX data
        unsigned long now = millis();
        lastDMXTime = now;
        
        // If motors were idle and DMX returns, trigger rehoming
        if (motorsIdle) {
          Serial.println("DMX signal restored - re-enabling motors and homing...");
          motorsIdle = false;
          stepperLED->enableOutputs();
          stepperRef->enableOutputs();
          needsRehoming = true;
        }

        dmx_read(dmxPort, dmxValues, packet.size);
        
        // Map DMX values to positions
        PanValues[0] = map((static_cast<uint16_t>(dmxValues[dmxStartAdresse]) << 8) | dmxValues[dmxStartAdresse + 1], 0, 65535, 0, MaxPos_LED);
        PanValues[1] = map((static_cast<uint16_t>(dmxValues[dmxStartAdresse + 2]) << 8) | dmxValues[dmxStartAdresse + 3], 0, 65535, 0, MaxPos_Ref);

        // Clamp positions
        PanValues[0] = constrain(PanValues[0], 0, MaxPos_LED);
        PanValues[1] = constrain(PanValues[1], 0, MaxPos_Ref);
        
        unsigned long frameInterval = (lastDMXReceiveTime == 0) ? DMX_FRAME_INTERVAL : (now - lastDMXReceiveTime);
        if (frameInterval == 0) {
          frameInterval = 1;
        }
        if (frameInterval > DMX_FRAME_INTERVAL * 4) {
          frameInterval = DMX_FRAME_INTERVAL;
        }

        portENTER_CRITICAL(&motionDataMux);
        previousPanValues[0] = currentPanValues[0];
        previousPanValues[1] = currentPanValues[1];
        currentPanValues[0] = PanValues[0];
        currentPanValues[1] = PanValues[1];
        dmxInterpolationInterval = frameInterval;
        lastDMXReceiveTime = now;
        // Store rotation speed in the same critical section as pan updates so
        // position task always sees a coherent frame snapshot.
        rotationSpeed[0] = dmxValues[dmxStartAdresse + 4];
        rotationSpeed[1] = dmxValues[dmxStartAdresse + 5];
        portEXIT_CRITICAL(&motionDataMux);
        
        // Handle rehoming after timeout
        if (needsRehoming) {
          needsRehoming = false;
          bool homingSuccess = homeBothAxes();
          
          if (homingSuccess) {
            rotationOffset[0] = 0;
            rotationOffset[1] = 0;
            Serial.println("Rehoming after timeout complete");
          } else {
            Serial.println("ERROR: Rehoming after timeout failed!");
          }
        }
        
        // Handle Reset / Homing from DMX channel
        if (dmxValues[dmxStartAdresse + 6] != 0) {
          bool homingSuccess = homeBothAxes();
          
          if (homingSuccess) {
            rotationOffset[0] = 0;
            rotationOffset[1] = 0;
          }
        }
        
      
      }
      taskYIELD();
    }
  }
}





//__________________________________Setup____________________________________________________
void setup() {
  Serial.begin(115200);
  
  // Disable WiFi and Bluetooth to save power and reduce heat
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.println("WiFi and Bluetooth disabled");
  
  //Fan pins
  pinMode(Fan_1, OUTPUT);
  pinMode(Fan_2, OUTPUT);
  //digitalWrite(Fan_1, HIGH);
  digitalWrite(Fan_2, LOW);
  digitalWrite(Fan_1, LOW);
  //Endstop Pins
  pinMode(Switch_1, INPUT);
  pinMode(Switch_2, INPUT);

  //Stepper Setup
  pinMode(StepperEnable, OUTPUT);
  digitalWrite(StepperEnable, LOW);
  Serial.println("Stepper drivers enabled");
  
  // Initialize FastAccelStepper Engine
  engine.init();
  Serial.println("FastAccelStepper engine initialized");
  
  // Create stepper instances
  stepperLED = engine.stepperConnectToPin(M1_Step);
  stepperRef = engine.stepperConnectToPin(M2_Step);
  
  if (stepperLED && stepperRef) {
    Serial.println("Stepper objects created successfully");
    
    // Configure LED stepper
    stepperLED->setDirectionPin(M1_Dir);
    stepperLED->setEnablePin(StepperEnable, true);
    stepperLED->setAutoEnable(false);  // Stepper bleibt permanent enabled
    stepperLED->setSpeedInHz(MaxSpeedLED);
    stepperLED->setAcceleration(AccelerationLED);
    stepperLED->setForwardPlanningTimeInMs(8);
    stepperLED->enableOutputs();  // WICHTIG: Stepper manuell enablen!
    Serial.print("LED Stepper configured - Speed: ");
    Serial.print(MaxSpeedLED);
    Serial.print(" Hz, Accel: ");
    Serial.println(AccelerationLED);
    
    // Configure Ref stepper
    stepperRef->setDirectionPin(M2_Dir);
    stepperRef->setEnablePin(StepperEnable, true);
    stepperRef->setAutoEnable(false);  // Stepper bleibt permanent enabled
    stepperRef->setSpeedInHz(MaxSpeedRef);
    stepperRef->setAcceleration(AccelerationRef);
    stepperRef->setForwardPlanningTimeInMs(8);
    stepperRef->enableOutputs();  // WICHTIG: Stepper manuell enablen!
    Serial.print("Ref Stepper configured - Speed: ");
    Serial.print(MaxSpeedRef);
    Serial.print(" Hz, Accel: ");
    Serial.println(AccelerationRef);
  } else {
    Serial.println("ERROR: Failed to create stepper objects!");
  }
  
  // Give steppers time to initialize
  delay(100);
  Serial.println("Stepper setup complete\n");

#if TEST_REF_MOTOR_ONLY
  // Isolated hardware test: move only the Ref axis back and forth forever,
  // completely bypassing DMX/homing so wiring/driver/motor can be verified alone.
  Serial.println("TEST_REF_MOTOR_ONLY active - oscillating Ref axis, all other logic skipped.");
  stepperRef->setSpeedInHz(MaxSpeedRef / 4);
  stepperRef->setAcceleration(AccelerationRef / 4);
  while (true) {
    Serial.print("Ref move to +20000, pos=");
    Serial.println(stepperRef->getCurrentPosition());
    stepperRef->moveTo(20000);
    while (stepperRef->isRunning()) { delay(1); }
    delay(300);

    Serial.print("Ref move to -20000, pos=");
    Serial.println(stepperRef->getCurrentPosition());
    stepperRef->moveTo(-20000);
    while (stepperRef->isRunning()) { delay(1); }
    delay(300);
  }
#endif
  
  //DMX Setup with RDM
  pinMode(Max485_TR, OUTPUT);
  digitalWrite(Max485_TR, LOW);
  
  // Configure DMX with RDM support
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  config.model_id = 1;  // Model ID for this fixture type
  config.product_category = RDM_PRODUCT_CATEGORY_FIXTURE;
  
  // Define personality for this fixture (7 DMX channels)
  dmx_personality_t personalities[] = {
    {7, "Pan/Pan fine Motor"}
  };
  const int personality_count = 1;
  
  // Install driver with RDM support
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
  
  // Set device label (already registered by driver, just update it)
  rdm_set_device_label(dmxPort, "2pan", 4);
  
  // Set DMX start address
  rdm_set_dmx_start_address(dmxPort, dmxStartAdresse);
  
  Serial.println("DMX initialized with RDM support");
  
  // Read back and verify RDM parameters
  char manufacturer[33] = {0};
  char model[33] = {0};
  char label[33] = {0};
  uint16_t dmx_addr = 0;
  
  rdm_get_manufacturer_label(dmxPort, manufacturer, sizeof(manufacturer));
  rdm_get_device_model_description(dmxPort, model, sizeof(model));
  rdm_get_device_label(dmxPort, label, sizeof(label));
  rdm_get_dmx_start_address(dmxPort, &dmx_addr);
  
  Serial.printf("RDM Parameters registered:\n");
  Serial.printf("  Manufacturer: %s\n", manufacturer);
  Serial.printf("  Model: %s\n", model);
  Serial.printf("  Device Label: %s\n", label);
  Serial.printf("  DMX Start Address: %d\n", dmx_addr);
  Serial.println("Motor control active");
  Serial.println();
  
  delay(100);
  
  // Create DMX reading task on core 0
  xTaskCreatePinnedToCore(
    dmxReadingTask,
    "DMX Task",
    4096,
    NULL,
    1,
    &dmxTask,
    0);

  //Homing-Sequenz mit separaten Hall-Sensoren für beide Achsen - BEFORE starting position task
  bool homingSuccess = homeBothAxes();
  
  // Start Position Task AFTER Homing completes, so steppers are at known positions
  xTaskCreatePinnedToCore(
    positionCalculationTask,  // Function that implements the task
    "Position Task",          // Name of the task
    4096,                     // Stack size in words
    NULL,                     // Task input parameter
    1,                        // Priority of the task
    &positionTask,            // Task handle
    1);                       // Run on core 1

  setupComplete = true; // Signal position task that it can start
  delay(100); // Brief pause to let task initialize
  
  if (!homingSuccess) {
    Serial.println("ERROR: Homing failed!");
  }
  
  // Accuracy test - uncomment to test homing repeatability
  // testHomingAccuracy(5);
}




void loop() {
  // FastAccelStepper runs automatically in background via hardware timers
  // No need to call run() anymore!
  
  static unsigned long lastTempPrint = 0;
  static unsigned long lastTimeoutCheck = 0;
  unsigned long now = millis();
  
  // Check for DMX timeout every second
  if (now - lastTimeoutCheck >= 1000) {
    lastTimeoutCheck = now;
    unsigned long timeSinceLastDMX = now - lastDMXTime;
    
    // If DMX timeout occurred and motors are not yet idle, disable them
    if (!motorsIdle && timeSinceLastDMX >= DMX_TIMEOUT_MS) {
      Serial.println("DMX timeout - disabling motors and entering idle mode");
      motorsIdle = true;
      stepperLED->disableOutputs();
      stepperRef->disableOutputs();
      Serial.printf("No DMX signal for %lu seconds\n", timeSinceLastDMX / 1000);
    }
  }
  
  // Print CPU temperature every 10 seconds
  if (now - lastTempPrint >= 10000) {
    float temp = temperatureRead();
    Serial.printf("CPU Temperature: %.1f°C", temp);
    
    // Track min/max
    if (!temperatureInitialized) {
      temperatureMin = temp;
      temperatureMax = temp;
      temperatureInitialized = true;
    } else {
      if (temp < temperatureMin) temperatureMin = temp;
      if (temp > temperatureMax) temperatureMax = temp;
    }
    
    if (temperatureInitialized) {
      Serial.printf(" (Low: %.1f°C, High: %.1f°C)", temperatureMin, temperatureMax);
    }
    Serial.println();
    lastTempPrint = now;
  }
  
  delay(10);
}