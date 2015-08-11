//defines.h 
/* This header file contains defined constants and macros used throughout the codebase */
#ifndef defines_h
#define defines_h 

//USER MODIFIABLE CONSTANTS//
#define SERIAL_BAUD_RATE 115200
#define PWM_32KHZ_PHASE //PWM Frequency for motor control
#define MOTORUPDATE_FREQ 1000 // in Hz, 1000 is default
#define ACC_LPF_FACTOR 3   //magnitude of Acc data lowpass, filtering increases Acc lag
#define GYROWEIGHT 0.990      // weight of gyro in complementary filter, out of 1 
#define PS_LPF_FACTOR 1
#define SUBTICK_FREQ 8
#define RCPIN1 4
#define MAX_VOLTAGE 1260 // in milli volts
#define ENABLE_VOLTAGE_COMPENSATION

//STATIC CONSTANTS// 
//Do not touch these unless you know what you are doing! 
#define PI 3.14159265        
#define PI_IN_THOUSANDTHS 3142 
#define RADTODEG 57.2957795  
#define RADTODEG_IN_THOUSANDTHS 57296 
#define GYROSCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //converts raw data units to radians, determined experimentally
#define dt 1000 // dt in micros seconds
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ     // loop control sample rate equals motor update rate
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)
#define N_SIN 256
#define BATT_INPUT_ADC_TO_MILLI_VOLTS 2.765
#endif
