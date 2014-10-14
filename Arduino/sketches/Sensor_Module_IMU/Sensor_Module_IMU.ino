#include <avr/eeprom.h>

static uint16_t calibratingG;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t gyroZero[3] = {0,0,0};
static uint16_t acc_1G;             // this is the 1G measured acceleration
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static int16_t  acc_25deg;


// ************************************************************************************************************
// Main Program
// ************************************************************************************************************
void setup()
{
  initSensors();
}

void loop()
{
	ACC_getADC();
	Gyro_getADC();
}
// ************************************************************************************************************
// Helper Functions
// ************************************************************************************************************