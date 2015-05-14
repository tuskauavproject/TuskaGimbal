#ifndef variables_h
#define variables_h

//Main
static bool outputAngle = false;
static float setAnglePitch = 0,setAnglePitchLPF = 0;
static float setAngleRoll = 0,setAngleRollLPF = 0;

static uint32_t prevPrint = 0;
static uint8_t subTick = 0;

// IMU
static int16_t  gyroADC[3],accADC[3]; // RawGyroData,RawAccData,LPAccData

// CONFIG VARIABLES
static int32_t pitchPID[3],rollPID[3]; 
static int8_t accDir[3] = {1,1,1};
static int8_t gyrDir[3] = {1,1,1};
static uint8_t pitchMotorPower = 0;

//RC Decode
volatile static byte stateRCInt = 0;    // a counter to see how many times the pin has changed
volatile static uint32_t lastTimeRising = 0;
volatile static uint16_t lastPulse = 0;
static int16_t rcPulse1 = 0;
#endif
