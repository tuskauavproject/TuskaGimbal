#ifndef Defines_h
#include"Defines.h"
#endif // !Defines_h

using namespace System::IO::Ports;
using namespace System;

public ref class Serial{
public:
	Serial(){
		_serialPort =  gcnew System::IO::Ports::SerialPort();
		this->portArray =  SerialPort::GetPortNames();
		this->comPort = L"NULL";
		this->baudRate = 0;
		this->comErrors = "";
		this->readTimeOut = READTIMEOUT;
		this->writeTimeout = WRITETIMEOUT;

		this->_serialPort->PortName = this->comPort;
		this->_serialPort->ReadTimeout = this->readTimeOut;
		this->_serialPort->WriteTimeout = this->writeTimeout;
		
	}

public: array<Object^>^ findPorts(){
			this->portArray = SerialPort::GetPortNames();
			return this->portArray;
		 }

		void connect(){
			this->comErrors = String::Empty;
			if(this->comPort == "NULL" || this->baudRate == 0)
				this->comErrors = "Please Select Port Settings";
			else {
				try{
					// make sure port isn't open	
				if(!this->_serialPort->IsOpen){
					this->_serialPort->PortName=this->comPort;
					
					this->_serialPort->BaudRate = this->baudRate;
					 
					this->comErrors = String::Empty;
					//open serial port 
					this->_serialPort->Open();
					this->connectionStatus = true;
					this->comErrors = "Port Opened"; 
				}
				else
					this->comErrors = "Port isn't openned";
				}
				catch(UnauthorizedAccessException^){
					this->comErrors = "UnauthorizedAccess";
				}
				_serialPort->ReadTimeout = this->readTimeOut;
			}
		}

		void disconect(){
			this->_serialPort->Close();
			this->connectionStatus=false;
			this->comErrors = "Port Closed"; 
		}

		bool sendCommand(String ^ message){
			if(this->connectionStatus)
			{
				message += "\r";
				this->_serialPort->Write(message);
				try{
					_serialPort->ReadLine();
				}
				catch(TimeoutException^){
					this->comErrors="Timeout Exception";
				}
				return true;
			}
			else
				return false;
		}

		void setBaudRate(int baud){
			this->baudRate = baud;
		}

		int getBaud(){
			return baudRate;
		}

		void setComPort(String^ cPort){
			this->comPort = cPort;
		}

		String ^getComPort(){
			return this->comPort;
		}

		String ^getComError(){
			return this->comErrors;
		}

		bool getStatus(){
		return this->connectionStatus;
		}
private:
	SerialPort^  _serialPort;
	array <String^>^ portArray;
	String^ comErrors;
	int baudRate;
	String^ comPort;
	bool connectionStatus;
	int readTimeOut,writeTimeout;
};