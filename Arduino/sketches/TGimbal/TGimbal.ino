//#include "Sensors.h"
#include "Timer1.h"
//#include "timer2.h"
#include "TuskaEEPROM.h"
#include "SerialCommand.h"
#include "variables.h"
#include "pinChangeInt.h"
#include "IMU.h"
#include <avr/pgmspace.h>

#define PWM_32KHZ_PHASE
#define PI 3.14159265         // approx of Pi which program uses.
#define dt 1000              // dt in micros seconds
#define GYROSCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) * 1 // A constant which converts raw data units to radians, determined experimentally
#define RADTODEG  57.2957795  //constant for converting radians to degrees
#define MOTORUPDATE_FREQ 1000                 // in Hz, 1000 is default
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ     // loop control sample rate equals motor update rate
//#define ACC_LPF_FACTOR 50   //magnitude of Acc data lowpass, filtering increases Acc lag
#define GYROWEIGHT 0.98       // weight of gyro in complementary filter, out of 1 
#define ACC_LPF_FACTOR 40

#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)

#define PS_LPF_FACTOR 100

#define RCPIN1 4

static float *pitch, *roll;
uint8_t freqCounter = 0;
bool motorUpdate = false; 

static int32_t pitchErrorSum = 0;
static int32_t rollErrorSum = 0;
static int32_t pitchErrorOld = 0;
static int32_t rollErrorOld = 0;
static int32_t rollInt = 0;
static int32_t pitchInt = 0;


static uint16_t calibratingG;         //used for initializing Acc calibration

static long calSum;                   //used for gyro calibration      10
static float gyroZero[3] = {0,0,0};   //stores values for gyro calibration
static unsigned long pTime = 0;                // variable for determining dt of integration, it will overflow at approx 70min
static float deltaGyroAngle[3] = {0,0,0}; //holds dø in degrees
static float angleSum[3]; // unusued variable for debug remove later

static uint16_t acc_1G;               // this is the 1G measured acceleration
static uint16_t calibratingA = 0;     // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static int16_t  acc_25deg;            // not sure what it is yet but we need it something to do with small angle estimations.
static float accelFloat[3] = {0,0,0};             // Converting int AccData to float AccData
static float accLPF[3];               // for Acc lowpass implementation

static float pitchAngle = 0;          // variable for final IMU pitch angle in deg
static float rollAngle = 0;           // variable for final IMU roll angle in deg
static float accVectorMagnitude = 0;

static float loopRate = 0;            //used for determining approx IMU sampling rate
static short loopCount = 1;
long prevTime = 0;

byte axis = 0;
int8_t pwmSinMotor[256];

TuskaEEPROM tEEPROM;
SerialCommand SCMD;


void setup(){
  Serial.begin(115200);
  pinMode(12,INPUT);
  pinMode(13,OUTPUT);
  digitalWrite(12,HIGH);

  configSerialCommands();
  tEEPROM.configEpprom();

  tEEPROM.readPID(pitchPID,rollPID);
  tEEPROM.initReadMotorPower(&pitchMotorPower);
        
	initSensors();
  //Serial.println("Calibrating MPU6050");
  for(axis = 0; axis < 3; axis++){ //calculate offsets to zeros gyros 
    for(int i = 0; i < 400; i++){ //averages 400 readings for each axis
      Gyro_getADC();
      calSum += gyroADC[axis];
      delay(1);
    }
	  gyroZero[axis] = calSum/400.f;
	  calSum = 0;
  }
  initBlController();
  initMotorStuff();
}

void loop(){
  SCMD.readSerial();
	Gyro_getADC(); // read raw gyro data
	ACC_getADC(); // read raw accel data
	

  pitchInt = pitchAngle *1000;
  rollInt = rollAngle *1000;
  
  int32_t pitchP = pitchPID[0];
  int32_t pitchI = pitchPID[1];
  int32_t pitchD = pitchPID[2];
  
  int32_t rollP = rollPID[0];
  int32_t rollI = rollPID[1];
  int32_t rollD = rollPID[2];
  
  setAnglePitchLPF = setAnglePitchLPF * (1.0f - (1.0f/PS_LPF_FACTOR)) + setAnglePitch * (1.0f/PS_LPF_FACTOR);
  setAngleRollLPF = setAngleRollLPF * (1.0f - (1.0f/PS_LPF_FACTOR)) + setAngleRoll * (1.0f/PS_LPF_FACTOR);
  
  //int pitchPID = ComputePID(DT_INT_MS, DT_INT_INV,-1*pitchInt, setAngle*1000, &pitchErrorSum, &pitchErrorOld,pitchP,pitchI,pitchD);//500,20,5,100 pwr,~10.7V
  int pitchPIDOutput = ComputePID(DT_INT_MS, DT_INT_INV,pitchInt, setAnglePitchLPF*1000, &pitchErrorSum, &pitchErrorOld,pitchP,pitchI,pitchD);//500,20,5,100 pwr,~10.7V
  int rollPIDOutput = ComputePID(DT_INT_MS, DT_INT_INV,-1*rollInt, setAngleRollLPF*1000, &rollErrorSum, &rollErrorOld,rollP,rollI,rollD);//500,20,5,100 pwr,~10.7V
  
  //MoveMotorPosSpeed(1, pitchPID, (uint16_t)pitchMotorPower);
  MoveMotorPosSpeed(0, pitchPIDOutput,60);
  MoveMotorPosSpeed(1, rollPIDOutput,90);
 
  if(subTick %8 == 0){
    if(outputAngle){
      Serial.print("PRA ");
      Serial.print(pitchAngle);
      Serial.print(" ");
      Serial.println(rollAngle);
    }
    //setAngle = (analogRead(A0) - 512)/10;
    //Serial.println(rollInt);
  }
  subTick++;
  subTick = subTick %256;
}

int32_t ComputePID(int32_t DTms, int32_t DTinv, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd)
{
  int32_t error = setPoint - in;
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTinv/10;
  *errorOld = error;

  out = out / 4096 / 8;

  return out;
  
}

inline int32_t constrain_int32(int32_t x , int32_t l, int32_t h){
  if (x <= l) {
    return l;
  } else if (x >= h) {
    return h;
  } else {
    return x;
  }
}
