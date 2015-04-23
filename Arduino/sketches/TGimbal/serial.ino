void configSerialCommands(){
  SCMD.addCommand("SPID",setPID);
  SCMD.addCommand("RPID",readPID);
  SCMD.addCommand("SMP",setMotorPower);
  SCMD.addCommand("RMP",readMotorPower);
  SCMD.addCommand("AST",startAngleSend);
  SCMD.addCommand("ASP",stopAngleSend);
}

void setPID(){
	char *axis;
	char *PID;
	char *val;
	float aNumber;
	axis = SCMD.next(); 
	PID = SCMD.next();
	val = SCMD.next();

	if (axis != NULL && PID != NULL && val != NULL) 
	{
		if(axis[0] == 'P'){
			aNumber=atof(val); 
			if(strcmp(PID,"0") == 0){
				tEEPROM.writeFloat("PitchP",aNumber);
			}
			else if(strcmp(PID,"1") == 0){
				tEEPROM.writeFloat("PitchI",aNumber);

			}
			else if(strcmp(PID,"2") == 0){
				tEEPROM.writeFloat("PitchD",aNumber);
			}
		}
		else if(axis[0] == 'R'){
			aNumber=atof(val); 
			if(strcmp(PID,"0") == 0){
				tEEPROM.writeFloat("RollP",aNumber);
			}
			else if(strcmp(PID,"1") == 0){
				tEEPROM.writeFloat("RollI",aNumber);

			}
			else if(strcmp(PID,"2") == 0){
				tEEPROM.writeFloat("RollD",aNumber);
			}
		}
                tEEPROM.readPID(pitchPID);

  	} 
  	else {
    	Serial.println("No argument SPID"); 
  	}
}

void readPID(){
	char *axis;
	char *PID;
	axis = SCMD.next();
	PID = SCMD.next();
	if (axis != NULL && PID != NULL) 
	{
		if(axis[0] == 'P'){
			if(strcmp(PID,"0") == 0){
                                Serial.print("SPID P 0 ");
				Serial.println(tEEPROM.readFloat("PitchP"));
			}
			else if(strcmp(PID,"1") == 0){
                                Serial.print("SPID P 1 ");
				Serial.println(tEEPROM.readFloat("PitchI"));

			}
			else if(strcmp(PID,"2") == 0){
                                Serial.print("SPID P 2 ");
				Serial.println(tEEPROM.readFloat("PitchD"));
			}
		}
		else if(axis[0] == 'R'){
			if(strcmp(PID,"0") == 0){
                                Serial.print("SPID R 0 ");
				Serial.println(tEEPROM.readFloat("RollP"));
			}
			else if(strcmp(PID,"1") == 0){
                                Serial.print("SPID R 1 ");
				Serial.println(tEEPROM.readFloat("RollI"));

			}
			else if(strcmp(PID,"2") == 0){
                                Serial.print("SPID R 2 ");
				Serial.println(tEEPROM.readFloat("RollD"));
			}
		}	
  	} 
  	else {
    	Serial.println("No argument"); 
  	}
}

void setMotorPower(){
  char *axis;
  char *val;
  uint8_t aNumber;
  
  axis = SCMD.next();
  val = SCMD.next();
  aNumber = atoi(val);
  if(axis[0] == 'P'){
      if(aNumber >= 0 && aNumber <= 256)
        tEEPROM.writeByte("MPowerP",aNumber);
  }
   else if(axis[0] == 'R'){
      if(aNumber >= 0 && aNumber <= 256)
        tEEPROM.writeByte("MPowerR",aNumber);
   }
   tEEPROM.initReadMotorPower(&pitchMotorPower);
}

void readMotorPower(){
  char *axis;
  axis = SCMD.next();
  if(axis[0] == 'P'){
    uint8_t mp = tEEPROM.readByte("MPowerP");
    Serial.println(mp);
    pitchMotorPower = mp;
  }
  else if(axis[0] == 'R'){
    uint8_t mp = tEEPROM.readByte("MPowerR");
    Serial.println(mp);
    pitchMotorPower = mp; 
  }
}

void startAngleSend(){
  outputAngle = true;
}

void stopAngleSend(){
  outputAngle = false;
}

