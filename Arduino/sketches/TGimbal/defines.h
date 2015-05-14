#ifndef defines_h
#define defines_h 
//defines.h 
/* This header file contains defined constants and macros used throughout the codebase */

//USER MODIFIABLE CONSTANTS//
#define PWM_32KHZ_PHASE //PWM Frequency for motor control
#define MOTORUPDATE_FREQ 1000 // in Hz, 1000 is default
#define ACC_LPF_FACTOR 50   //magnitude of Acc data lowpass, filtering increases Acc lag
#define GYROWEIGHT 0.98       // weight of gyro in complementary filter, out of 1 
#define PS_LPF_FACTOR 100
#define RCPIN1 4


//STATIC CONSTANTS//
#define PI 3.14159265        
#define PI_IN_THOUSANDTHS 3142 
#define RADTODEG 57.2957795  
#define RADTODEG_IN_THOUSANDTHS 57296 
// A constant which converts raw data units to radians, determined experimentally
#define GYROSCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) * 1 
#define dt 1000 // dt in micros seconds
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ     // loop control sample rate equals motor update rate
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)

#endif
