#ifndef variables_h
#define variables_h
#include "defines.h"
//MAIN
static float pitchAngle = 0;          // variable for final IMU pitch angle in deg
static float rollAngle = 0;           // variable for final IMU roll angle in deg

static bool outputAngle = false;
static float setAnglePitch = 0,setAnglePitchLPF = 0;
static float setAngleRoll = 0,setAngleRollLPF = 0;

static uint32_t prevPrint = 0;
static uint8_t subTick = 0;

//SETUP 
static uint16_t calibratingG;         //used for initializing Acc calibration
static long calSum;                   //used for gyro calibration      10
static float gyroZero[3] = {0,0,0};   //stores values for gyro calibration

// IMU
static int16_t  gyroADC[3],accADC[3]; // RawGyroData,RawAccData,LPAccData

//SENSOR
static uint16_t acc_1G;               // this is the 1G measured acceleration
static uint16_t calibratingA = 0;     // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static int16_t  acc_25deg;            // not sure what it is yet but we need it something to do with small angle estimations.
static float accelFloat[3] = {0,0,0};             // Converting int AccData to float AccData
static float accLPF[3];               // for Acc lowpass implementation

// PID ALGORITHM 
static int32_t pitchErrorSum = 0;
static int32_t rollErrorSum = 0;
static int32_t pitchErrorOld = 0;
static int32_t rollErrorOld = 0;
static int32_t rollInt = 0;
static int32_t pitchInt = 0;

// CONFIG VARIABLES
static int32_t pitchPID[3],rollPID[3]; 
static int8_t accDir[3] = {1,1,1};
static int8_t gyrDir[3] = {1,1,1};
static uint8_t pitchMotorPower = 0;

//RC DECODE
static volatile byte stateRCInt = 0;    // a counter to see how many times the pin has changed
static volatile uint32_t lastTimeRising = 0;
static volatile uint16_t lastPulse = 0;
static int16_t rcPulse1 = 0;

//BRUSHLESS MOTOR CONTROL
static int8_t pwmSinMotor[256];
static uint8_t freqCounter = 0;
static bool motorUpdate = false; 
#endif
