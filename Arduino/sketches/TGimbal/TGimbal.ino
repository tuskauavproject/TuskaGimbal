#include "Timer1.h"
#include "TuskaEEPROM.h"
#include "SerialCommand.h"
#include "variables.h"
#include "pinChangeInt.h"
#include "IMU.h"
#include "defines.h"
#include <avr/pgmspace.h>

TuskaEEPROM tEEPROM;
SerialCommand SCMD;
IMU imu;

void setup(){

  configSerialCommands();
  tEEPROM.configEpprom();

  tEEPROM.readPID(pitchPID,rollPID);
  tEEPROM.initReadMotorPower(&pitchMotorPower,&rollMotorPower);
  tEEPROM.initReadStabilize(&stabilizePitch,&stabilizeRoll);

	initSensors();
  imu.setup();
  gyroCalibration();
  
  initBlController();
  initMotorStuff();
}

void loop(){
  SCMD.readSerial();
	Gyro_getADC(); // read raw gyro data
	ACC_getADC(); // read raw accel data
  imu.calculate(gyroADC,accADC,&pitchAngle,&rollAngle);
  
  pitchAngularVelocity = gyroADC[GYRO_X_AXIS];
  rollAngularVelocity = gyroADC[GYRO_Y_AXIS];

  int32_t pitchP = pitchPID[0];
  int32_t pitchI = pitchPID[1];
  int32_t pitchD = pitchPID[2];
 
  int32_t rollP = rollPID[0];
  int32_t rollI = rollPID[1];
  int32_t rollD = rollPID[2];
  
  setAnglePitchLPF = setAnglePitchLPF * (1.0f - (1.0f/PS_LPF_FACTOR)) + setAnglePitch * (1.0f/PS_LPF_FACTOR);
  setAngleRollLPF = setAngleRollLPF * (1.0f - (1.0f/PS_LPF_FACTOR)) + setAngleRoll * (1.0f/PS_LPF_FACTOR);

  desiredAngularVelocityPitch = stabilizePitch * (pitchAngle - setAnglePitchLPF);
  desiredAngularVelocityRoll = stabilizeRoll * (-1*rollAngle - setAngleRollLPF);

  int pitchPIDOutput = ComputePID(DT_INT_MS, DT_INT_INV,pitchAngularVelocity, desiredAngularVelocityPitch, &pitchErrorSum, &pitchErrorOld,pitchP,pitchI,pitchD);//500,20,5,100 pwr,~10.7V
  int rollPIDOutput = ComputePID(DT_INT_MS, DT_INT_INV,rollAngularVelocity, desiredAngularVelocityRoll, &rollErrorSum, &rollErrorOld,rollP,rollI,rollD);//500,20,5,100 pwr,~10.7V
  #ifdef ENABLE_VOLTAGE_COMPENSATION
    int correctedPitchMotorPower =(int) (MAX_VOLTAGE/(float)inputMillivolts * pitchMotorPower); 
    int correctedRollMotorPower = (int) (MAX_VOLTAGE/(float)inputMillivolts * rollMotorPower);

    MoveMotorPosSpeed(pitchMotorNumber, pitchPIDOutput,correctedPitchMotorPower);
    MoveMotorPosSpeed(rollMotorNumber, rollPIDOutput,correctedRollMotorPower);
    
  #else
    MoveMotorPosSpeed(pitchMotorNumber, pitchPIDOutput,pitchMotorPower);
    MoveMotorPosSpeed(rollMotorNumber, rollPIDOutput,rollMotorPower);
  #endif
  
  MoveMotorPosSpeed(pitchMotorNumber, pitchPIDOutput,pitchMotorPower);
  MoveMotorPosSpeed(rollMotorNumber, rollPIDOutput,rollMotorPower);
 
  if(subTick % SUBTICK_FREQ == (SUBTICK_FREQ-1)){
    inputMillivolts = analogRead(A0) * BATT_INPUT_ADC_TO_MILLI_VOLTS;

    if(outputAngle){
      Serial.print("PRA ");
      Serial.print(pitchAngle);
      Serial.print(" ");
      Serial.println(rollAngle);
      subTick = 0;
    }
  }
  subTick++;
}

int32_t ComputePID(int32_t DTms, int32_t DTinv, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd){
  int32_t error = setPoint - in;
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;

  Ierr = constrain_int32(Ierr, -(int32_t)500000, (int32_t)500000);

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

void pin7Changed(){
  Serial.println("Changed");
}
