//IMU.cpp
#include "IMU.h"
//current execution time is 832 microSeconds
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
  	deltaGyroAngle[axis] = (gyroSample[axis]) * gyrDir[axis]  * GYROSCALE * (currentTimeInMicroseconds-previousTimeInMicroseconds) * RADTODEG * 0.875; // (dø/dt - calibration) * dt * constatnt 
  }
}

void IMU::complimentaryFilter(){
	gyroPitchSum += deltaGyroAngle[GYRO_X_AXIS];
	pitchAngle += deltaGyroAngle[GYRO_X_AXIS]; //riemann sum
    rollAngle += deltaGyroAngle[GYRO_Y_AXIS]; //riemann sum

	if(accVectorMagnitude < UPPER_ACC_LIMIT && accVectorMagnitude > LOWER_ACC_LIMIT){ //Acc vector must be <1.3g && >0.7g, else discard reading
		pitchAccel = _atan2(accFiltered[ACC_Y_AXIS] * accDir[ACC_Y_AXIS],accFiltered[ACC_Z_AXIS] * accDir[ACC_Z_AXIS]); // determine the pitch of Acc vector using atan2 function
    	rollAccel = _atan2(accFiltered[ACC_X_AXIS] * accDir[ACC_X_AXIS],sqrt(pow(accFiltered[ACC_Y_AXIS],2) + pow(accFiltered[ACC_Z_AXIS],2)));  // determine the roll of Acc vector using atan2 function
    	
    	if(pitchAccel == 0) // there is a bug where sometimes _atan2 returns 0 this is a crude fix but the problem is rare
    		pitchAccel == pitchAngle * 10;
    	if(rollAccel == 0)
    		rollAngle == rollAngle * 10;
    		
		pitchAngle = pitchAngle * GYROWEIGHT + pitchAccel * ACCEL_WEIGHT; // complementary filter 
		rollAngle = rollAngle * GYROWEIGHT + rollAccel * ACCEL_WEIGHT;    // complementary filter 
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

void IMU::setup(){
	#ifdef REVERSE_X_AXIS
		gyrDir[GYRO_X_AXIS] = -1;
	#endif 

	#ifdef REVERSE_Y_AXIS
		gyrDir[GYRO_Y_AXIS] = -1;
	#endif

	#ifdef REVERSE_Z_AXIS
		gyrDir[GYRO_Z_AXIS] = -1;
	#endif

	#ifdef REVERSE_X_AXIS_ACC
		accDir[ACC_X_AXIS] = -1;
	#endif

	#ifdef REVERSE_Y_AXIS_ACC
		accDir[ACC_Y_AXIS] = -1;
	#endif

	#ifdef REVERSE_Z_AXIS_ACC
		accDir[ACC_Z_AXIS] = -1;
	#endif
}

int16_t IMU::_atan2(float y, float x){
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}