// UNCOMMENT ONE OF THE FOUR OPTIONS BELOW

#define PRINTDATA // uncomment to display raw data over serial port in format [gX     gY     gZ      aX      aY      aZ]
//#define PRINTANGLE // identical to PRINTDATA but gyro data is in delta degrees [gX     gY     gZ      aX      aY      aZ]
//#define PRINTANGLESUM // uncomment to display simple intigration of gyro over X axis in degrees
//#define SAMPLERATE // uncomment to display max sampling rate over serial port updates every 2 seconds with sampleing rate in Hz

#include <avr/eeprom.h>

static uint16_t calibratingG;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t gyroZero[3] = {0,0,0};
static uint16_t acc_1G;             // this is the 1G measured acceleration
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static int16_t  acc_25deg;

#if defined(PRINTANGLE) || defined(PRINTANGLESUM)
	#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))
	#define RADTODEG  57.2957795;
#endif

// ************************************************************************************************************
// Main Program
// ************************************************************************************************************
void setup()
{
  initSensors();
  Serial.begin(115200);
}

void loop()
{	    	
	ACC_getADC();
	Gyro_getADC();
	#if (defined(PRINTANGLE) || defined(PRINTANGLESUM))
		static uint16_t previousT;
 		uint16_t currentT = micros();
 		float scale, deltaGyroAngle[3];
 		float static angleSum[3];
 		byte axis = 0;

 		scale = (currentT - previousT) * GYRO_SCALE;
 		previousT = currentT;

 		for (axis = 0; axis < 3; axis++){
    		deltaGyroAngle[axis] = gyroADC[axis]  * scale * RADTODEG;
    	}

    #if defined(PRINTANGLESUM)
    	angleSum[0] += deltaGyroAngle[0];
    	if(Serial.available() > 0)
    	{
    		angleSum[0] = 0;
    		Serial.read();
    	}
    	Serial.println(angleSum[0]);
   	#endif

   	#if defined(PRINTANGLE)
    	printAngleData(deltaGyroAngle);
    #endif

	#endif
	#if defined(PRINTDATA) 
		printRawData();
	#elif defined(SAMPLERATE)
		printSampleRate();
	#endif
	
}
// ************************************************************************************************************
// Helper Functions
// ************************************************************************************************************
#if defined(PRINTDATA)
	void printRawData() //prints raw data over serial port in format "[gX     gY     gZ      aX      aY      aZ]\n"
	{
		Serial.print("[");
		Serial.print(gyroADC[0]);
		Serial.print("\t\t");
		Serial.print(gyroADC[1]);
		Serial.print("\t\t");
		Serial.print(gyroADC[2]);
		Serial.print("\t\t");
		Serial.print(accADC[1]);
		Serial.print("\t\t");
		Serial.print(accADC[2]);
		Serial.print("\t\t");
		Serial.print(accADC[3]);
		Serial.println("]");
	}
#endif

#if defined(PRINTANGLE)
	void printAngleData(float *deltaGyroAngle){
		Serial.print("[");
		Serial.print(deltaGyroAngle[0]);
		Serial.print("\t\t");
		Serial.print(deltaGyroAngle[1]);
		Serial.print("\t\t");
		Serial.print(deltaGyroAngle[2]);
		Serial.print("\t\t");
		Serial.print(accADC[1]);
		Serial.print("\t\t");
		Serial.print(accADC[2]);
		Serial.print("\t\t");
		Serial.print(accADC[3]);
		Serial.println("]");
	}
#endif

#if defined(SAMPLERATE)
		long ptime = 0;
		int count = 0;
	void printSampleRate() //takes number of samples over approx 2 sec. and prints out the sampling rate in Hz to serial port.
	{
		count++;
		if((millis() - ptime) >= 2000 ){
			Serial.print("Sampling Rate:");
			Serial.print((float)count/((millis() - ptime)/ (float)1000));
			Serial.println("Hz");
			ptime = millis();
			count = 0;
		}
	}
#endif