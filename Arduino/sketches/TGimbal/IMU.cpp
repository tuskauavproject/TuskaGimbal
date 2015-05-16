//IMU.cpp
#include "IMU.h"

void IMU::calculate(int16_t gyroADC[3],int16_t accADC[3],float *pitch,float* roll){
	setGyroSample(gyroADC);
	setAccSample(accADC);
	pitchAngle = *pitch;
	rollAngle = *roll;
	accLowPassFilter();

  	currentTimeInMicroseconds = microsT1(); // set current time 
  	calculateDeltaGyroAngle();     
    previousTimeInMicroseconds = currentTimeInMicroseconds;
    calculateAccVectorMagnitude();
    complimentaryFilter();
    *pitch = pitchAngle;
    *roll = rollAngle;
}

void IMU::lowPassFilter(int sampleValue,float* filteredValue, int lowPassFilterFactor){
	*filteredValue = ((1.0 - (1.0/lowPassFilterFactor)) * (*filteredValue)) + ((1.0/lowPassFilterFactor) * sampleValue);
}

void IMU::accLowPassFilter(){
	for(uint8_t axis = 0;axis < 3; axis++)
		lowPassFilter(accSample[axis],&accFiltered[axis],ACC_LPF_FACTOR);
}

void IMU::calculateAccVectorMagnitude(){
	accVectorMagnitude = pow(accFiltered[0],2) + pow(accFiltered[1],2) + pow(accFiltered[2],2);
}

void IMU::calculateDeltaGyroAngle(){
  for (uint8_t axis = 0; axis < 3; axis++){
  	deltaGyroAngle[axis] = (gyroSample[axis]) * gyrDir[axis]  * GYROSCALE * (currentTimeInMicroseconds-previousTimeInMicroseconds) * RADTODEG; // (dø/dt - calibration) * dt * constatnt 
  }
}

void IMU::complimentaryFilter(){
	pitchAngle -= deltaGyroAngle[GYRO_X_AXIS]; //riemann sum
    rollAngle += deltaGyroAngle[GYRO_Y_AXIS]; //riemann sum

	if(accVectorMagnitude < UPPER_ACC_LIMIT && accVectorMagnitude > LOWER_ACC_LIMIT){ // if !bullshit Acc vector must be <1.3g && >0.7g
		pitchAccel = atan2(accFiltered[ACC_Y_AXIS],accFiltered[ACC_Z_AXIS]) * RADTODEG; // determine the pitch of Acc vector using atan2 function
    	rollAccel = atan2(accFiltered[ACC_X_AXIS],accFiltered[ACC_Z_AXIS]) * RADTODEG;  // determine the roll of Acc vector using atan2 function

		pitchAngle = pitchAngle * GYROWEIGHT + pitchAccel * (1 - GYROWEIGHT); // complementary filter 
		rollAngle = rollAngle * GYROWEIGHT + rollAccel * (1 - GYROWEIGHT);    // complementary filter 
	}  
}

void IMU::setGyroSample(int *x){
	for(uint8_t axis = 0; axis < 3; axis++)
		gyroSample[axis] = x[axis];
}

void IMU::setAccSample(int *x){
	for(uint8_t axis = 0; axis < 3; axis++)
		accSample[axis] = x[axis];
}

