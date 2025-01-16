#include <AccelStepper.h>

class Motor {
    long PanValue;
    long currentMotorPosition;
    long tarValue;
    long maxMotorPosition;
    long minMotorPosition;
    long maxSpeed;
    long microstepping;
    unsigned long lastChange;
    unsigned long dmxdelaymid;
    AccelStepper stepper;
    byte EndPin;
    bool EndstopNS = true;
    bool NormalDir = true;

    public:
        Motor(byte dirPin, byte stepPin, byte EndStopPin, bool invertMotorDir, unsigned long StepperMaxSpeed, unsigned long StepperAcceleration, unsigned long Microstepping,
        unsigned long maxPos, unsigned long minPos) : stepper(1,stepPin,dirPin)
        {
            microstepping = Microstepping;
            maxMotorPosition = maxPos*microstepping;
            minMotorPosition = minPos*microstepping;
            maxSpeed = StepperMaxSpeed*microstepping;
            stepper.setMaxSpeed(maxSpeed);
            stepper.setAcceleration(StepperAcceleration*microstepping);
            NormalDir = invertMotorDir;
            stepper.setPinsInverted(invertMotorDir, false, false); // Invert direction pin
            EndPin=EndStopPin;
            pinMode(EndStopPin, INPUT);



        }

        void update(){
            stepper.run();
        }
        
        bool Rotary_homing(){
            bool b = true;
            stepper.setMaxSpeed(maxSpeed/2);
            stepper.move(-maxMotorPosition*6/10);
            while(b) { //move while endstop is not closed (low = closed)
                stepper.run();
            
                if (EndstopNS){
                    b = digitalRead(EndPin);  
                }else{
                    b = !digitalRead(EndPin);
                }

                if(stepper.distanceToGo()==0){
                    return false;
                }
            }
            delay(200);
            stepper.setCurrentPosition(0);
            //check if it is the end
            stepper.runToNewPosition(-50*microstepping);
            if (EndstopNS){
                b = digitalRead(EndPin);  
            }else{
                b = !digitalRead(EndPin);
            }
            delay(200);
            stepper.move(-maxMotorPosition*6/10);
            while(b) { //move next round (not the end) while endstop is not closed (low = closed)
                stepper.run();
            
                if (EndstopNS){
                    b = digitalRead(EndPin);  
                }else{
                    b = !digitalRead(EndPin);
                }

                if(stepper.distanceToGo()==0){
                    return false;
                }
            }
            stepper.setCurrentPosition(0);
            //check if it is the end
            stepper.runToNewPosition(minMotorPosition);
            while(b) {
                stepper.move(-1);
                stepper.run();
                if (EndstopNS){
                    b = digitalRead(EndPin);  
                }else{
                    b = !digitalRead(EndPin);
                }
            }
            stepper.setCurrentPosition(0);
            stepper.runToNewPosition(minMotorPosition);
            stepper.setCurrentPosition(0);
            delay(10);
            return true;
        }

        bool Switch_homing(){
            bool b = true;
            stepper.setMaxSpeed(maxSpeed/2);
            stepper.move(-maxMotorPosition*11/10);
            while(b) { //move while endstop is not closed (low = closed)
                stepper.run();
            
                if (EndstopNS){
                    b = digitalRead(EndPin);  
                }else{
                    b = !digitalRead(EndPin);
                }

                if(stepper.distanceToGo()==0){
                    return false;
                }
            }
            stepper.setCurrentPosition(0);
            stepper.runToNewPosition(minMotorPosition);
            b = true;
            while(b) {
                stepper.move(-1);
                stepper.run();
                if (EndstopNS){
                    b = digitalRead(EndPin);  
                }else{
                    b = !digitalRead(EndPin);
                }
            }
            stepper.setCurrentPosition(0);
            stepper.runToNewPosition(minMotorPosition);
            stepper.setCurrentPosition(0);
            delay(10);
            return true;
        }

        void setStepperPosandSpeed(unsigned long dmxDel, long PanV, long TarV){
            if (dmxDel < 20) {
                dmxDel = 20; // Set a minimum delay
            }
            long stepsToMoveA = abs(PanV-TarV);
            long stepsToMoveB = abs(stepper.distanceToGo());
            unsigned long speed = (stepsToMoveA+stepsToMoveB)*1000/dmxDel;
            if (speed < maxSpeed && speed > 100) {
                stepper.setMaxSpeed(speed);
            } else if (speed <= 1) {
                stepper.setMaxSpeed(1);
            } else {
                stepper.setMaxSpeed(maxSpeed);
            }
            stepper.moveTo(PanV);
        }
        long mapSafe(long long value, long long in_min, long long in_max, long long out_min, long long out_max) {
            if (in_max == in_min) {
                // Avoid division by zero
                //Serial.println("Error: Division by zero in map");
                return out_min;
            }
            long long result = out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);

            // Ensure the result is constrained within the output range
            if (result < out_min) result = out_min;
            if (result > out_max) result = out_max;

            return result;
        }
        
        void setDMX(byte pan, byte panFine){
            unsigned long now = millis(); 
            unsigned long dmxDelay = now-lastChange;
            lastChange = now;
            dmxdelaymid = dmxdelaymid*0.99+dmxDelay*0.01;
            unsigned long Value = (static_cast<uint16_t>(pan) << 8) | panFine;
            PanValue = mapSafe(Value,0,65535,minMotorPosition,maxMotorPosition);
            if(tarValue != PanValue){
                //long stepsToMoveA = abs(tarValue-PanValue);
                stepper.moveTo(PanValue);
                /*if (i==0){
                    long stepsToMoveB = abs(stepper.distanceToGo());
                    u_int16_t speed = (stepsToMoveA+stepsToMoveB)*1000/dmxdelaymid;
                    
                if (speed < maxSpeed && speed > 100) {
                    stepper.setMaxSpeed(speed);
                } else if (speed <= 100) {
                    stepper.setMaxSpeed(100);
                } else {
                    stepper.setMaxSpeed(maxSpeed);
                }
                    stepper.moveTo(PanValue);
                }else{
                    setStepperPosandSpeed(dmxdelaymid,PanValue,tarValue);
                    //targetPos[i]=PanValues[i];
                }*/
                tarValue = PanValue;
            }
        }
};
