#include <avr/eeprom.h>       //libray required for eeprom I/O
//#include "Adafruit_GFX.h"
//#include "Adafruit_SSD1306.h" //libray required for I2C display

#include <SPI.h>             //libray required for I2C display

static uint16_t calibratingG;         //used for initializing Acc calibration
static int16_t  gyroADC[3],accADC[3],accSmooth[3]; // RawGyroData,RawAccData,LPAccData

static long calSum;                   //used for gyro calibration       
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
#define GYROWEIGHT 0.90       // weight of gyro in complementart filter, out of 1  
#define SCREENREFRESH 60      // screen refresh rate in Hz, increasing too much will lag IMU
#define ACC_LPF_FACTOR 10
static float *pitch, *roll;

/*
MOTOR STUFF
*/
#define SIN_ARRAY_SIZE 255
#define PI 3.14159265


byte sinArray[SIN_ARRAY_SIZE];
byte pinA = 9;
byte pinB = 10;
byte pinC = 11;
int tStep = 0;
byte pwmA,pwmB,pwmC = 0;
int phaseOffset1,phaseOffset2 = 0;
uint32_t pastTime = 0;
int8_t dir = -1;
int8_t motorDirection = -1;
volatile uint8_t motorUpdateTick = 0;
volatile uint8_t motorUpdate = 0;
volatile uint8_t motorSpeedUpdateTick = 0;
volatile uint8_t motorSpeed = 50;

void setupInterupt()
{
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  //TCCR2A = 0;// set entire TCCR2A register to 0
  //TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 255;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  //TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();//allow interrupts
}

// ************************************************************************************************************
// Main Program
// ************************************************************************************************************
void setup()
{
  initSensors();        //initilizes sensors for sepecifics see Sensors.ino
  delay(100);
  Serial.begin(115200); //initilize serial communication with 115200 baudrate
  setupInterupt();
  delay(100);
  calcSinArray();

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

  for(axis = 0; axis < 3; axis++)
    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
    //accelFloat[axis] = accLPF[axis]; // raw data from int array to float array also give the option to filter if necessary 
  
    static uint16_t previousT;
    uint16_t currentT = micros(); // set current time 
    float scale, deltaGyroAngle[3]; 
     
    scale = (currentT - previousT) * GYROSCALE; // scalar for raw data GYROSCALE converts raw data to rad/sec then * dt
    previousT = currentT; 

    for (axis = 0; axis < 3; axis++){
        deltaGyroAngle[axis] = (gyroADC[axis])  * scale * RADTODEG; // (dø/dt - calibration) * dt * constatnt 
      }
  
    angleSum[0] += deltaGyroAngle[0]; // for debug remove!!!!!!!!!!!!! 

    pitchAngle += deltaGyroAngle[0]; //riemann sum
    rollAngle -= deltaGyroAngle[1]; //riemann sum  
    

    float pitchAccel = atan2(accLPF[1],accLPF[2]) * RADTODEG; // determine the pitch of Acc vector using atan2 function
    float rollAccel = atan2(accLPF[0],accLPF[2]) * RADTODEG;  // determine the roll of Acc vector using atan2 function

    accVectorMagnitude = sqrt(pow(accADC[0],2) + pow(accADC[1],2) + pow(accADC[2],2))/495.f; // calc of of Acc vector magnitude, could be simplified 

    if(accVectorMagnitude < 1.3 && accVectorMagnitude > 0.7) // if !bullshit Acc vector must be <1.3g && >0.7g
    {
      if(!(abs(rollAngle) > 85))
        pitchAngle = pitchAngle * GYROWEIGHT + pitchAccel * (1 - GYROWEIGHT); // complementary filter 
      if(!(abs(pitchAngle) > 85))
        rollAngle = rollAngle * GYROWEIGHT + rollAccel * (1 - GYROWEIGHT);    // complementary filter 
    }

    else // for debuging purposes only triggered if Acc magnitude is out of bounds
    {
      //Serial.print("THROWOUT!!!!!!!!!!!!!!!!!!!!!!!!!!");
      //Serial.print("\t\t");
    }

    if(pitchAngle > 180)
      pitchAngle = -180 + (pitchAngle - 180);
    else if(pitchAngle < -180)
      pitchAngle = 180 + (pitchAngle + 180);

    if(rollAngle > 180)
      rollAngle = -180 + (rollAngle - 180);
    else if(rollAngle < -180)
      rollAngle = 180 + (rollAngle + 180);

    serialPrintDebug(pitchAccel); //prints to Serial for debug
    if(motorUpdate == 1)
  {
    motorUpdate = 0;
    runMotor(tStep);
  } 
}
// ************************************************************************************************************
// Helper Functions
// ************************************************************************************************************

int16_t _atan2(float y, float x){
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

/*void writeOLED() // function that creates OLED display 
{
  display.drawCircle(64,32,30,1);

  int xCord = (30 * cos(rollAngle/RADTODEG));   // for drawing horizon line
  int yCord = (30 * sin(rollAngle/RADTODEG));   // for drawing horizon line
  int yHud  = map(pitchAngle,-100,100,-30,30);  // for y postion of pitch indicator 

  display.drawLine(64,32,64 - xCord,32 + yCord,1); // draws half of horizon line
  display.drawLine(64,32,64 + xCord,32 - yCord,1); // draws other half of horizon line
  display.drawFastHLine(59,32 - yHud ,10,1);       // draws horizontal line for pitch indicator

  display.setCursor(90,0);            //displays pitchAngle top right
  display.println(pitchAngle);

  display.setCursor(92,50);           //displays IMU sampling estimation bottom right
  display.print(loopRate,0);
  display.println("Hz");

  display.setCursor(0,50);            //displays roll angle bottom left
  display.println(-rollAngle);

  display.setCursor(0,0);             //displays Acc Vector Magnitude top left
  display.print(accVectorMagnitude);
  display.println("g");

  display.display();                  //call for display
  display.clearDisplay();             //clears buffer for next display
}*/

/*void displayInit()    
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //init I2C display with adress 0x3C

  display.setTextColor(1); //settings for printing text
  display.setTextSize(1);
  display.setTextWrap(0);

  display.clearDisplay();
  display.drawBitmap(0,17,tuskalogo,127,34,1); //displays beautiful tuska logo 
  display.display();
}*/

void serialPrintDebug(float pitchAccel)
{
if(Serial.available() > 0)
      {
        angleSum[0] = 0;
        angleSum[1] = 0;
        angleSum[2] = 0;
        Serial.read();
      }

      Serial.println(pitchAngle);
      //Serial.print("\t\t");
      //Serial.print(deltaGyroAngle[0]);
      //Serial.print("\t\t");
      //Serial.print(pitchAccel);
      //Serial.print("\t\t");
      //Serial.print(pitchAngle);
      //Serial.print("\t\t");
      //Serial.println(rollAngle);
      //Serial.println(accVectorMagnitude);
      //Serial.println(pitchAngle);
}

void calcSinArray(void)
{
  for(int i = 0; i < SIN_ARRAY_SIZE; i++)
  {
    sinArray[i] = (SIN_ARRAY_SIZE)/2.f * sin(2 * PI/float(SIN_ARRAY_SIZE) * i - PI/2.f) + (SIN_ARRAY_SIZE)/2.f;
  }
  phaseOffset1 = SIN_ARRAY_SIZE/360.f * 120;
  phaseOffset2 = SIN_ARRAY_SIZE/360.f * 240;
}

void runMotor(uint8_t Step)
{
  pwmA = sinArray[Step];
  pwmB = sinArray[(Step + phaseOffset1) % SIN_ARRAY_SIZE];
  pwmC = sinArray[(Step + phaseOffset2) % SIN_ARRAY_SIZE];
  controlWithImu();
}

ISR(TIMER2_COMPA_vect){//timer1 interrupt 64 kHz 
  motorUpdateTick++;
  motorSpeedUpdateTick++;
  if(motorUpdateTick == 10)
  {
    motorUpdate = 1;
    analogWrite(pinA, pwmA);
    analogWrite(pinB, pwmB);
    analogWrite(pinC, pwmC);
    motorUpdateTick = 0; 
    
  }
  if(motorSpeedUpdateTick == motorSpeed)
  {
    if(motorDirection == 1)
      tStep++;
    else
      tStep--;
    motorSpeedUpdateTick = 0;
  }
}

void controlWithImu()
{
  if(pitchAngle > 0)
    motorDirection = -1;
  else
    motorDirection = 1;
  motorSpeed = map(abs(pitchAngle),0,90,1000,100);
}
