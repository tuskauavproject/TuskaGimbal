#ifndef variable_h
#define variable_h
//Main
bool outputAngle = false;
float setAngle = 0;
uint32_t prevPrint = 0;
uint8_t subTick = 0;

// CONFIG VARIABLES
int32_t pitchPID[3]; 
int8_t accDir[3] = {1,1,1};
int8_t gyrDir[3] = {1,1,1};
uint8_t pitchMotorPower = 0;

//RC Decode
volatile byte stateRCInt = 0;    // a counter to see how many times the pin has changed
volatile uint32_t lastTimeRising = 0;
volatile uint16_t lastPulse = 0;
int16_t rcPulse1 = 0;
#endif
