//IMU.h
#ifndef TuskaEEPROM_h
#define TuskaEEPROM_h

#include "Arduino.h"
#include "variables.h"
#include "Timer1.h"

#define GYRO_X_AXIS 0
#define GYRO_Y_AXIS 2
#define GYRO_Z_AXIS 1

#define ACC_X_AXIS 0
#define ACC_Y_AXIS 2
#define ACC_Z_AXIS 1

#define ACC_LPF_FACTOR 40
#define GYROSCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) * 1
#define RADTODEG  57.2957795  //constant for converting radians to degrees
#define UPPER_ACC_LIMIT 422500 //1.3g  = (1.3*500)^2 
#define LOWER_ACC_LIMIT 122500 //0.7g  = (0.7*500)^2 
#define GYROWEIGHT 0.98 

class IMU
{
public:
	void calculate();

private:
	void lowPassFilter(int sampleValue,float* filteredValue, int lowPassFilterFactor);
	void accLowPassFilter();
	void calculateDeltaGyroAngle();
	void calculateAccVectorMagnitude();
	void complimentaryFilter();

	int accSample[3];
	float accFiltered[3];
	
  	float deltaGyroAngle[3]; 

  	static uint32_t previousTimeInMicroseconds,currentTimeInMicroseconds;

  	float gyroScaleFactor;

  	float pitchAccel,rollAccel;
  	float pitchAngle,rollAngle;
  	int accVectorMagnitude;

};

#endif 