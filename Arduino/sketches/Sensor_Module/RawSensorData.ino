// UNCOMMENT ONE OF THE TWO OPTIONS BELOW

//#define PRINTDATA // uncomment to display raw data over serial port in format [gX     gY     gZ      aX      aY      aZ]
#define SAMPLERATE // uncomment to display sampling rate over serial port updates every 2 seconds with sampleing rate in Hz

#include <avr/eeprom.h>
// some of these variables probably are not useful eventually we should try and remove them.
static uint16_t calibratingG;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t gyroZero[3] = {0,0,0};
static uint16_t acc_1G;             // this is the 1G measured acceleration
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
static int16_t  acc_25deg;


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
	#if defined(PRINTDATA) 
		printRawData();
	#elif defined(SAMPLERATE)
		printSampleRate();
	#endif
	
}

// ************************************************************************************************************
// Helper Functions
// ************************************************************************************************************
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