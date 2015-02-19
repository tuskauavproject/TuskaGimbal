#include <avr/eeprom.h>       //libray required for eeprom I/O

static uint16_t calibratingG;         //used for initializing Acc calibration
static int16_t  gyroADC[3],accADC[3],accSmooth[3]; // RawGyroData,RawAccData,LPAccData

static long calSum;                   //used for gyro calibration      10
static float gyroZero[3] = {0,0,0};   //stores values for gyro calibration
static long pTime = 0;                // variable for determining dt of integration, it will overflow at approx 70min
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

byte axis = 0;                        // used in for loops for array index 

#define PI 3.14159265         // approx of Pi which program uses.
#define dt 10000              // dt in micros seconds
#define GYROSCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) * 1 // A constant which converts raw data units to radians, determined experimentally
#define RADTODEG  57.2957795  //constant for converting radians to degrees
//#define ACC_LPF_FACTOR 50   //magnitude of Acc data lowpass, filtering increases Acc lag
#define GYROWEIGHT 0.93       // weight of gyro in complementart filter, out of 1  
#define SCREENREFRESH 60      // screen refresh rate in Hz, increasing too much will lag IMU
#define ACC_LPF_FACTOR 10
static float *pitch, *roll;

// ************************************************************************************************************
// Motor Control and PID
// ************************************************************************************************************

#define N_SIN 256
#define CC_FACTOR 32000 // 32kHz timer
#define MOTORUPDATE_FREQ 500 //Motor update will be called at 500Hz (every 2ms)

//phase angle offsets for 2 of 3 motor phases
#define phaseAngle1 360.f/N_SIN * 120
#define phaseAngle2 360.f/N_SIN * 240

#define DT_FLOAT (1.0/LOOPUPDATE_FREQ*1.024) // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)    // dT, integer, (ms)
#define DT_INT_INV (MOTORUPDATE_FREQ)        // dT, integer, inverse, (Hz)

//Abstraction of PWM Motors Pins 9-11 and 3,5,6
//Do not change unless you know what you are doing see atmega 328p datasheet
#define PWM_A_MOTOR1 OCR2A   
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

int8_t pwmSinMotor[256];   // size of sin lookup table to drive motors
volatile byte motorPos0 = 0;
volatile byte motorPos1 = 0;
volatile byte motorPos2 = 0;
volatile byte motorDir1 = 1;
volatile uint8_t mainTick = 0;
uint8_t subTick = 0;
volatile uint8_t motorSpeed1 = 0;
volatile uint8_t motorSpeed10 = 1;
volatile uint16_t freqCounter0 = 0;
volatile uint16_t freqCounter1 = 0;
volatile uint16_t freqCounter2 = 0;
volatile uint16_t motorCounter0 = 10;
volatile uint16_t motorCounter1 = 10;


volatile bool motorUpdate = false;  //enabled when freqCounter = CC_FACTOR/MOTORUPDATE_FREQ
//rotation direction for select motor
int8_t pitchDirection = 1; 
int8_t rollDirection = 1;

volatile uint8_t freqCounter = 0; //tracks number of interupt calls to enable motorUpdate
//variables which drive index of sinArray 
int pitchMotorDrive = 0;
int rollMotorDrive = 0;

// control motor update in ISR
bool enableMotorUpdates = true; //

//variables that hold PWM duty cycle
uint8_t pwm_a_motor0 = 128;
uint8_t pwm_b_motor0 = 128;
uint8_t pwm_c_motor0 = 128;

uint8_t pwm_a_motor1 = 128;
uint8_t pwm_b_motor1 = 128;
uint8_t pwm_c_motor1 = 128;

//The PID values which control the motor
int32_t pitchPIDVal;
int32_t rollPIDVal;


float pitchAngleSet = -25; //Angle to hold the gimbal at in degrees

//variables for PID calculations
static int32_t pitchErrorSum = 0;
static int32_t rollErrorSum = 0;
static int32_t pitchErrorOld = 0;
static int32_t rollErrorOld = 0;

int i = 0;
int inc = 1;

// ************************************************************************************************************
// Main Program
// ************************************************************************************************************
void setup()
{
  initSensors();        //initilizes sensors for sepecifics see Sensors.ino
  delay(100);
  Serial.begin(115200); //initilize serial communication with 115200 baudrate
  delay(100);
  initBlController();
  // Init Sinus Arrays
  initMotorStuff();

  for(axis = 0; axis < 3; axis++) //calculate offsets to zeros gyros 
  {
	  for(int i = 0; i < 400; i++) //averages 400 readings for each axis
	  {
	  	Gyro_getADC();
	  	calSum += gyroADC[axis];
      delayMicroseconds(650);
	  }
	  gyroZero[axis] = calSum/400.f;
	  calSum = 0;
  }

  //delay(2000); // so people can see sick logo ;)
}

void loop()
{
  Gyro_getADC(); // read raw gyro data
  ACC_getADC(); // read raw accel data
  IMU();
  if (mainTick)
  {
    i = i + inc;
    if(i == 0|| i ==500)
      inc = inc * -1;
   // Serial.print(i);
    // set pitch motor pwm
    pitchPIDVal = ComputePID(DT_INT_MS, DT_INT_INV,1000*pitchAngle,pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld,35, 20, 5); //4.5, 1.5, 0.4
    //Serial.print(pitchAngle);
    if (pitchPIDVal > 0)
      motorDir1 = 1;
    else 
      motorDir1 = -1;
    motorSpeed1 = abs(pitchPIDVal);
    //motorSpeed10 = (motorSpeed % 10) + 1;
    //motorSpeed1 = motorSpeed / 10;
    //Serial.print("\t");
    //Serial.println(pitchPIDVal);
    setMotorSpeed(1,10,250); 
    subTick++;
    mainTick = false;
  }
  if(subTick == 10)
  {
   pitchAngleSet = map(analogRead(A0),0,1024,-70,70);
   //Serial.println(pitchAngleSet);
   subTick = 0;
  }
}

// ************************************************************************************************************
// Helper Functions
// ************************************************************************************************************



