//IMU.h
#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include "variables.h"
#include "Timer1.h"
#include "defines.h"

#define GYRO_X_AXIS 0
#define GYRO_Y_AXIS 2
#define GYRO_Z_AXIS 1

#define ACC_X_AXIS 0
#define ACC_Y_AXIS 2
#define ACC_Z_AXIS 1

#define UPPER_ACC_LIMIT 422500 //1.3g  = (1.3*500)^2 
#define LOWER_ACC_LIMIT 122500 //0.7g  = (0.7*500)^2 

typedef struct orientation
{

};

class IMU
{
public:
	void calculate(int16_t gyroADC[3],int16_t accADC[3]);

private:
	void lowPassFilter(int sampleValue,float* filteredValue, int lowPassFilterFactor);
	void accLowPassFilter();
	void calculateDeltaGyroAngle();
	void calculateAccVectorMagnitude();
	void complimentaryFilter();
	void setGyroSample(int *x);
	void setAccSample(int *x);

	int accSample[3],gyroSample[3];
	float accFiltered[3];
  	float deltaGyroAngle[3]; 
  	uint32_t previousTimeInMicroseconds,currentTimeInMicroseconds;
  	float gyroScaleFactor;
  	float pitchAccel,rollAccel;
  	float pitchAngle,rollAngle;
  	uint32_t accVectorMagnitude;
};

#endif 