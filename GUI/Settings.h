#ifndef DEFINES_H
#include "Defines.h"
#endif // !DEFINES_H

public ref class Settings{

public:
	//default constructor
	Settings(){
		rollPID = gcnew array<double>(3);
        rollPID[0] = RPDefault;
		rollPID[1] = RIDefault;
		rollPID[2] = RDDefault;

		pitchPID = gcnew array<double>(3);
        pitchPID[0] = PPDefault;
		pitchPID[1] = PIDefault;
		pitchPID[2] = PDDefault;
		
		this->motorSelect=true;
		this->rollMotorDirection=true;
		this->pitchMotorDirection=true;
		this->swapXYAxis=false;
		this->reverseXAxis=false;
		this->reverseYAxis=false;
		this->reverseZAxis=false;
		this->pitchMotorPower = 127;
		this->rollMotorPower = 127;
	}

	Settings(array<double> ^RPID,array<double> ^PPID,bool MS,bool RMD,bool PMD,bool SXY,bool RXA,bool RYA,bool RZA){
		this->rollPID = RPID;
		this->pitchPID = PPID;
		this->motorSelect=MS;
		this->rollMotorDirection=RMD;
		this->pitchMotorDirection=PMD;
		this->swapXYAxis=SXY;
		this->reverseXAxis=RXA;
		this->reverseYAxis=RYA;
		this->reverseZAxis=RZA;
	}
	Settings(Settings ^ a){
		this->rollPID = gcnew array<double>(3);
		this->pitchPID = gcnew array<double>(3);
		for(int i=0; i < a->rollPID->Length; i++)
			this->rollPID[i]=a->rollPID[i];
		for(int i=0; i < a->pitchPID->Length; i++)
			this->pitchPID[i]=a->pitchPID[i];
		this->motorSelect=a->motorSelect;
		this->rollMotorDirection=a->rollMotorDirection;
		this->pitchMotorDirection=a->pitchMotorDirection;
		this->swapXYAxis=a->swapXYAxis;
		this->reverseXAxis=a->reverseXAxis;
		this->reverseYAxis=a->reverseYAxis;
		this->reverseZAxis=a->reverseZAxis;
		this->pitchMotorPower = a->pitchMotorPower;
		this->rollMotorPower= a->rollMotorPower;
	}

	bool setRollPID(int i, double val){
		if(i < rollPID->Length && i >= 0){
			if(val >= PMIN && val<= PMAX)
				rollPID[i] = val;
			else
				return true;
			return false;
		}
		return true;
	}

	/*bool setPitchPID(float PVal,float IVal,float DVal){
		if(PMIN <= PVal && PVal<= PMAX)
			pitchPID[P]=(int)(PVal*1000);
		else
			return true;
		if(IMIN <= IVal && IVal <= IMAX)
			pitchPID[I]=(int)(IVal*1000);
		else
			return true;
		if(DMIN <= DVal && DVal <= DMAX)
			pitchPID[D]=(int)(DVal*1000);
		else
			return true;
		return false;
	}*/

	bool setPitchPID(int i, double val){
		if(i < pitchPID->Length && i >= 0){
			if(val >= PMIN && val<= PMAX)
				pitchPID[i] = val;
			else
				return true;
			return false;
		}
		return true;
		
	}

	float getRollPID(int index){
		return rollPID[index];
	}

	array<double> ^ getRollPID(void){
		return rollPID;
	}

	float getPitchPID(int index){
		return pitchPID[index];
	}

	array<double> ^ getPitchPID(void){
		return pitchPID;
	}
	
	void setMotorSelect(bool b){
		motorSelect = b;
	}

	bool getMotorSelect(){
		return motorSelect;
	}

	void setRollMotorDirection(bool b){
		rollMotorDirection = b;
	}

	void setPitchMotorDirection(bool b){
		pitchMotorDirection = b;
	}

	bool getRollMotorDirection(){
		return rollMotorDirection;
	}

	bool getPitchMotorDirection(){
		return pitchMotorDirection;
	}

	int getPitchMotorPower(void){
		return this->pitchMotorPower;
	}

	bool setPitchMotorPower(int p){
		if(p >= 0 && p < 256)
			pitchMotorPower = p;
		else
			return true; 
		return false;
	}

	int getRollMotorPower(){
		return this->rollMotorPower;
	}

	bool setRollMotorPower(int p){
		if(p >= 0 && p < 256)
			rollMotorPower = p;
		else
			return true; 
		return false;
	}

	void setSwapXYAxis(bool b){
		swapXYAxis = b;
	}

	bool getSwapXYAxis(){
		return swapXYAxis;
	}

	void setReverseXAxis(bool b){
		reverseXAxis = b;
	}

	bool getReverseXAxis(){
		return reverseXAxis;
	}

	void setReverseYAxis(bool b){
		reverseYAxis = b;
	}

	bool getReverseYAxis(){
		return reverseYAxis;
	}

	void setReverseZAxis(bool b){
		reverseZAxis = b;
	}

	bool getReverseZAxis(){
		return reverseZAxis;
	}
	
	array<bool> ^getRollPIDChanged(){
		return this->rollPIDChanged;
	}

	bool getRollPIDChanged(int index){
		if(index > this->rollPIDChanged->Length && index < 0){
			System::Console::WriteLine("getRollPIDChanged error index out of range!");
			return false;
		}
		else{
			return this->rollPIDChanged[index];
		}
	}

	void setRollPIDChanged(array<bool> ^a){
		this->rollPIDChanged = a;
	}

	void setRollPIDChanged(int i,bool val){
		if(i < this->rollPIDChanged->Length && i > 0)
			rollPIDChanged[i] = val;
	}

	array<bool> ^getPitchPIDChanged(){
		return this->pitchPIDChanged;
	}

	bool getPitchPIDChanged(int index){
		if(index > this->pitchPIDChanged->Length && index < 0){
			System::Console::WriteLine("getRollPIDChanged error index out of range!");
			return false;
		}
		else{
			return this->pitchPIDChanged[index];
		}
	}

	void setPitchPIDChanged(array<bool> ^a){
		this->pitchPIDChanged = a;
	}

	void setPitchPIDChanged(int i,bool val){
		if(i < this->pitchPIDChanged->Length && i > 0)
			pitchPIDChanged[i] = val;
	}

	bool getMotorSelectChanged(){
		return this->motorSelectChanged;
	}

	void setMotorSelectChanged(bool a){
		this->motorSelectChanged = a;
	}
	// motor power changed
	bool getRollMotorPowerChanged(){
		return rollMotorPowerChanged;
	}
	void setRollMotorPowerChanged(bool a){
		this->rollMotorPowerChanged = a;
	}

	bool getPitchMotorPowerChanged(){
		return pitchMotorPowerChanged;
	}
	void setPitchMotorPowerChanged(bool a){
		this->pitchMotorPowerChanged = a;
	}

	// motor direction changed 

	bool getRollMotorDirectionChanged(){
		return this-> rollMotorDirectionChanged;
	}
	void setRollMotorDirectionChanged(bool a){
		this->rollMotorDirectionChanged = a;
	}

	bool getPitchMotorDirectionChanged(){
		return this->pitchMotorDirectionChanged;
	}
	void setPitchMotorDirectionChanged(bool a){
		this->pitchMotorDirectionChanged = a;
	}
	
	// axis changed

	bool getSwapXYAxisChanged(){
		return this->swapXYAxisChanged;
	}
	void setSwapXYAxisChanged(bool a){
		this->swapXYAxisChanged = a;
	}

	bool getReverseXAxisChanged(){
		return this->reverseXAxisChanged;
	}
	void setReverseXAxisChanged(bool a){
		this->reverseXAxisChanged = a;
	}

	bool getReverseYAxisChanged(){
		return this->reverseYAxisChanged;
	}
	void setReverseYAxisChanged(bool a){
		this->reverseYAxisChanged = a;
	}

	bool getReverseZAxisChanged(){
		return this->reverseZAxisChanged;
	}
	void setReverseZAxisChanged(bool a){
		this->reverseZAxisChanged = a;
	}


private:
	array<double> ^rollPID;
	array<double> ^pitchPID;
	bool motorSelect;
	int rollMotorPower,pitchMotorPower;
	bool rollMotorDirection,pitchMotorDirection;
	bool swapXYAxis,reverseXAxis,reverseYAxis,reverseZAxis;

	array<bool> ^rollPIDChanged;
	array<bool> ^pitchPIDChanged;
	bool motorSelectChanged;
	bool rollMotorPowerChanged,pitchMotorPowerChanged;
	bool rollMotorDirectionChanged,pitchMotorDirectionChanged;
	bool swapXYAxisChanged,reverseXAxisChanged,reverseYAxisChanged,reverseZAxisChanged;
};