//TuskaEPPROM.cpp

#include "TuskaEEPROM.h"

void TuskaEEPROM::configEpprom(){
	EEPROM.setMaxAllowedWrites(800);
	uint16_t currentAddress = 0;
	for (uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char type[8];

		strcpy(type,eppromSaveValuesList[i].type);
		if(strcmp(type, "Byte")  == 0){
			strcpy(eppromTable[i].name,eppromSaveValuesList[i].name);
			strcpy(eppromTable[i].type,eppromSaveValuesList[i].type);
			eppromTable[i].address = currentAddress;
			currentAddress += 1;
		}
		else if(strcmp(type, "Int")  == 0){
			strcpy(eppromTable[i].name,eppromSaveValuesList[i].name);
			strcpy(eppromTable[i].type,eppromSaveValuesList[i].type);
			eppromTable[i].address = currentAddress;
			currentAddress += 2;
		}
		else if(strcmp(type, "Float")  == 0){
			strcpy(eppromTable[i].name,eppromSaveValuesList[i].name);
			strcpy(eppromTable[i].type,eppromSaveValuesList[i].type);
			eppromTable[i].address = currentAddress;
			currentAddress += 4;
		}	
	} 	
}

int TuskaEEPROM::writeByte(char* findName,uint8_t data){
	bool found = false;
	uint16_t address = 0;
	char type[6];

	for(uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char curName[8];
		strcpy(curName,eppromTable[i].name);
		if(strcmp(findName,curName) == 0){
			found = true;
			address = eppromTable[i].address;
			strcpy(type,eppromTable[i].type);
			break;
		}
	}
	if(found && strcmp(type,"Byte") == 0){
		EEPROM.updateByte(address,data);
		return 0;
	}
	else{
		return -1;
	}
}

int TuskaEEPROM::writeInt(char* findName,int data){
	bool found = false;
	uint16_t address = 0;
	char type[6];

	for(uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char curName[8];
		strcpy(curName,eppromTable[i].name);
		if(strcmp(findName,curName) == 0){
			found = true;
			address = eppromTable[i].address;
			strcpy(type,eppromTable[i].type);
			break;
		}
	}
	if(found && strcmp(type,"Int") == 0){
		EEPROM.updateInt(address,data);
		return 0;
	}
	else{
		return -1;
	}
}

int TuskaEEPROM::writeFloat(char* findName,float data){
	bool found = false;
	uint16_t address = 0;
	char type[6];

	for(uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char curName[8];
		strcpy(curName,eppromTable[i].name);
		if(strcmp(findName,curName) == 0){
			found = true;
			address = eppromTable[i].address;
			strcpy(type,eppromTable[i].type);
			break;
		}
	}
	if(found && strcmp(type,"Float") == 0){
		EEPROM.updateFloat(address,data);
		return 0;
	}
	else{
		return -1;
	}
}




uint8_t TuskaEEPROM::readByte(char*findName){
	bool found = false;
	uint16_t address = 0;
	char type[6];

	for(uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char curName[8];
		strcpy(curName,eppromTable[i].name);
		if(strcmp(findName,curName) == 0){
			found = true;
			address = eppromTable[i].address;
			strcpy(type,eppromTable[i].type);
			break;
		}
	}
	if(found && strcmp(type,"Byte") == 0){
		return EEPROM.readByte(address);
	}
	else{
		return 0;
	}
}

int TuskaEEPROM::readInt(char*findName){
	bool found = false;
	uint16_t address = 0;
	char type[6];

	for(uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char curName[8];
		strcpy(curName,eppromTable[i].name);
		if(strcmp(findName,curName) == 0){
			found = true;
			address = eppromTable[i].address;
			strcpy(type,eppromTable[i].type);
			break;
		}
	}
	if(found && strcmp(type,"Int") == 0){
		return EEPROM.readInt(address);
	}
	else{
		return 0;
	}
}

float TuskaEEPROM::readFloat(char*findName){
	bool found = false;
	uint16_t address = 0;
	char type[6];

	for(uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char curName[8];
		strcpy(curName,eppromTable[i].name);
		if(strcmp(findName,curName) == 0){
			found = true;
			address = eppromTable[i].address;
			strcpy(type,eppromTable[i].type);
			break;
		}
	}

	if(found && strcmp(type,"Float") == 0){
		return EEPROM.readFloat(address);
	}
	else{
		return 0;
	}
}

void TuskaEEPROM::readPID(int32_t*PPID,int32_t*RPID){
  int p = readFloat("PitchP");
  int i = readFloat("PitchI");
  int d = readFloat("PitchD");
  
  PPID[0] = p;
  PPID[1]= i;
  PPID[2] = d;
  
  p = readFloat("RollP");
  i = readFloat("RollI");
  d = readFloat("RollD");
  
  RPID[0] = p;
  RPID[1]= i;
  RPID[2] = d;
}

void TuskaEEPROM::initReadMotorPower(uint8_t* pwrP,uint8_t* pwrR){
   uint8_t pwr = readByte("MPowerP");
   *pwrP = pwr;

   pwr = readByte("MPowerR");
   *pwrR = pwr;
}

void TuskaEEPROM::initReadStab(int16_t* stabP,int16_t* stabR){
	uint16_t stab = readInt("StabP");
	*stabP = stab;

	stab = readInt("StabR");
	*stabR = stab;
}
