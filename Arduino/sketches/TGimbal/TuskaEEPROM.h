//TuskaEPPROM.h
#ifndef TuskaEEPROM_h
#define TuskaEEPROM_h

#include "EEPROMex.h"
#include <avr/pgmspace.h>

#define NUMBER_EEPROM_ELEMENTS 8

typedef struct{
	char name[8];
	char type[8];
}eepromLookUpBlock;

typedef struct{
	char name[8];
	uint16_t address;
	char type[8];
}eepromBlock;

#define FS(x) (__FlashStringHelper*)(x)
const char PROGMEM text[]="hello";

class TuskaEEPROM 
{
  public:
    void configEpprom(void);
    int writeByte(char* name,uint8_t data);
    //int writeByte(uint8_t adress, uint8_t data);

    int writeInt(char*name,int data);
    //int writeByte(uint8_t address, int16_t data);

    int writeFloat(char*name,float data);
    //int writeFloat(uint8_t address,float data);

    uint8_t readByte(char* name);
    int readInt(char* name);
    float readFloat(char* name);
    
    void readPID(int32_t*PPID,int32_t*RPID); 
    void initReadMotorPower(uint8_t*PWR);
    
  private: 								// a more elegant method of epprom layout but could not get PROGMEM working with new avr-libc,
  	const char Float[6] PROGMEM = "Float";	// without PROGMEM it uses too much SRAM
	const  eepromLookUpBlock eppromSaveValuesList[NUMBER_EEPROM_ELEMENTS] PROGMEM = {
		{"PitchP","Float"},
		{"PitchI","Float"},
		{"PitchD","Float"},
		{"RollP","Float"},
		{"RollI","Float"},
		{"RollD","Float"},
		{"MPowerP","Byte"},
		{"MPowerR","Byte"},
	};
	eepromBlock eppromTable[NUMBER_EEPROM_ELEMENTS] PROGMEM;

};
//////////////////EPPROM LAYOUT/////////////////////////////
//					  address		type (0 = byte, 1=int16, 2 =float)

/*
#define pitchP_ADRESS 	0			//2
#define pitchP_TYPE	  				2

#define pitchI_ADRESS 	4
#define pitchI_TYPE	  				2

#define pitchD_ADRESS 	8
#define pitchD_TYPE	  				2

#define MPower1_ADRESS 	12
#define MPower1_TYPE	  			0

#define MPower2_ADRESS	13			
#define MPower2_TYPE				0
*/


#endif
