//TuskaEPPROM.cpp

#include "TuskaEEPROM.h"

void TuskaEEPROM::configEpprom(){
	uint16_t currentAddress = 0;
	for (uint16_t i = 0; i < NUMBER_EEPROM_ELEMENTS; i++){
		char type[8];

		strcpy(type,eppromSaveValuesList[i].type);
		//Serial.println(type);
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
		//else
			//Serial.println("Fail");
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
		//Serial.print("Not Found");
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
		//Serial.print("Not Found");
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
		//Serial.print("Not Found");
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
		//Serial.print("Not Found");
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
		//Serial.print("Not Found");
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
		//Serial.print("Not Found");
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
  
  /*Serial.print("Pitch\tP:");
  Serial.print(PID[0]);
  Serial.print("\tI:");
  Serial.print(PID[1]);
  Serial.print("\tD:");
  Serial.println(PID[2]);*/
}

void TuskaEEPROM::initReadMotorPower(uint8_t* pwrP){
   uint8_t pwr = readByte("MPowerP");
   *pwrP = pwr;
}
