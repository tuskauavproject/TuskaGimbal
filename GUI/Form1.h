#pragma once
#include "Settings.h"
#include <math.h>

#ifndef Defines_h
#include "Defines.h"
#endif // !DEFINES_H

#ifndef Serial_h
#include "Serial.h"
#endif

namespace GimbalGui20 {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Threading;

	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			Form1::DoubleBuffered = true;
			InitializeComponent();
			backBuffer = gcnew Bitmap(ClientSize.Width, ClientSize.Height);
			//frontBuffer = gcnew Bitmap(ClientSize.Width, ClientSize.Height);
			//gBackBuffer->
			gBackBuffer = Graphics::FromImage(backBuffer);
			referenceSettings = gcnew Settings();
			currentSettings = gcnew Settings(); //use default constructor for now
			SerialCom = gcnew Serial();
			resetGui();
			SerialProcessing = gcnew Thread(gcnew ThreadStart(this, &Form1::processSerialCommands));
			SerialProcessing->Start();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}

	public:
		Settings^ currentSettings;
		Settings^ referenceSettings;
		Serial^ SerialCom;
		Thread^ SerialProcessing;
		Thread^ mouseDown;
	private:
		UInt64 mainTick;
		bool updateAngles;
		double pitchDisplay,rollDisplay;
		double pitchAngleSet,rollAngleSet;
		Bitmap^ backBuffer;
		Graphics^ gBackBuffer;

	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::Label^  labelRollAngle;
	protected: 

	private: System::Windows::Forms::Label^  labelPitchAngle;

	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::CheckBox^  checkBoxReverseY;

	private: System::Windows::Forms::CheckBox^  checkBoxReverseX;
	private: System::Windows::Forms::CheckBox^  checkBoxReverseZ;


	private: System::Windows::Forms::CheckBox^  checkBoxSwapXY;

	private: System::Windows::Forms::Button^  bnCalAcc;
	private: System::Windows::Forms::Button^  bnStopReadingAngle;

	private: System::Windows::Forms::Button^  bnStartReadingAngle;

	private: System::Windows::Forms::Button^  bnCalGyro;
	private: System::Windows::Forms::StatusStrip^  statusStrip1;
	private: System::Windows::Forms::ToolStripStatusLabel^  toolStripStatusLabel1;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Label^  comErrors;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::ProgressBar^  progressBar1;
	private: System::Windows::Forms::Button^  closeButton;
	private: System::Windows::Forms::ComboBox^  comboBoxComPort;
	private: System::Windows::Forms::Button^  initButton;
	private: System::Windows::Forms::ComboBox^  comboBoxBaud;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::GroupBox^  pMotorGroup;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::TextBox^  pDBox;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::TextBox^  pIBox;
	private: System::Windows::Forms::TextBox^  pPBox;
	private: System::Windows::Forms::GroupBox^  groupBoxPitchMotorDirection;

	private: System::Windows::Forms::RadioButton^  radioPitchReverse;

	private: System::Windows::Forms::RadioButton^  radioPitchForward;

	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::TextBox^  textBoxPitchMotorPower;

	private: System::Windows::Forms::TrackBar^  trackBarPitchMotorPower;


	private: System::Windows::Forms::Label^  title;
	private: System::Windows::Forms::GroupBox^  groupBox6;
	private: System::Windows::Forms::GroupBox^  groupBox7;
	private: System::Windows::Forms::RadioButton^  radioButton1;
	private: System::Windows::Forms::RadioButton^  radioButton2;
	private: System::Windows::Forms::GroupBox^  groupBox8;
	private: System::Windows::Forms::RadioButton^  radioButton5;
	private: System::Windows::Forms::RadioButton^  radioButton6;
	private: System::Windows::Forms::GroupBox^  groupBox9;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::TextBox^  textBox4;
	private: System::Windows::Forms::GroupBox^  groupBox11;
	private: System::Windows::Forms::TextBox^  textBox5;
	private: System::Windows::Forms::TrackBar^  trackBar2;
private: System::Windows::Forms::GroupBox^  groupBoxPitchMotorSelect;

	private: System::Windows::Forms::RadioButton^  radioPitchMotor2;

	private: System::Windows::Forms::RadioButton^  radioPitchMotor1;

	private: System::Windows::Forms::GroupBox^  groupBox12;
private: System::Windows::Forms::GroupBox^  groupBoxRollMotorDirection;

	private: System::Windows::Forms::RadioButton^  radioRollReverse;

	private: System::Windows::Forms::RadioButton^  radioRollForward;
private: System::Windows::Forms::GroupBox^  groupBoxRollMotorSelect;


	private: System::Windows::Forms::RadioButton^  radioRollMotor2;

	private: System::Windows::Forms::RadioButton^  radioRollMotor1;

	private: System::Windows::Forms::GroupBox^  groupBox15;
private: System::Windows::Forms::TextBox^  rDBox;

	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::Label^  label16;
private: System::Windows::Forms::TextBox^  rIBox;

private: System::Windows::Forms::TextBox^  rPBox;

	private: System::Windows::Forms::GroupBox^  groupBox16;
private: System::Windows::Forms::TextBox^  textBoxRollMotorPower;

private: System::Windows::Forms::TrackBar^  trackBarRollMotorPower;
private: Microsoft::VisualBasic::PowerPacks::ShapeContainer^  shapeContainer2;
private: System::Windows::Forms::Timer^  timer1;
private: System::Windows::Forms::Button^  buttonReset;
private: System::Windows::Forms::Button^  buttonLoad;
private: System::Windows::Forms::Button^  buttonSave;
private: System::Windows::Forms::Button^  buttonDefault;
private: System::Windows::Forms::PictureBox^  pictureBox1;
private: System::Windows::Forms::BindingSource^  bindingSource1;


private: Microsoft::VisualBasic::PowerPacks::RectangleShape^  rectangleShape1;
private: Microsoft::VisualBasic::PowerPacks::ShapeContainer^  shapeContainer3;
private: System::Windows::Forms::Label^  labelSetAngleRoll;
private: System::Windows::Forms::Label^  labelSetAnglePitch;




private: System::ComponentModel::IContainer^  components;


private: bool compare(bool a, bool b){
		if(a != b)
			return true;
		else
			return false;
	}

	int compare(int a, int b)
	{
		if(a!=b)
			return true;
		else
			return false;
	}

	int compare(float a, float b, float threshold)
	{
		if(abs(a-b) <= threshold)
			return true;
		else
			return false;
	}

	array<bool> ^ compare(array<double>^ arrayA, array<double> ^arrayB){
		array<bool> ^ arrayReturn = gcnew array<bool>(3);
		 for(int i=0; i<3; i++)
            arrayReturn[i] = false;

		if(arrayA->Length != arrayB->Length){
			for(int i=0; i < arrayReturn->Length;i++)
				arrayReturn[i] = false;
		}
		else{
			for(int i=0; i < arrayA->Length; i++){
				if(abs(arrayA[i]-arrayB[i]) < (arrayA[i] + arrayB[i])/2.f * pow(10.f,-6))
					arrayReturn[i] = false;
				else
					arrayReturn[i] = true;
			}
		}
		return arrayReturn;
	}

	void checkForChanges(Settings ^newSettings, Settings ^oldSettings){
		newSettings->setRollPIDChanged(compare(newSettings->getRollPID(),oldSettings->getRollPID()));
		newSettings->setPitchPIDChanged(compare(newSettings->getPitchPID(),oldSettings->getPitchPID()));
		newSettings->setMotorSelectChanged(compare(newSettings->getMotorSelect(),oldSettings->getMotorSelect()));
		newSettings->setRollMotorPowerChanged(compare(newSettings->getRollMotorPower(),oldSettings->getRollMotorPower()));
		newSettings->setRollMotorDirectionChanged(compare(newSettings->getRollMotorDirection(),oldSettings->getRollMotorDirection()));
		newSettings->setPitchMotorPowerChanged(compare(newSettings->getPitchMotorPower(),oldSettings->getPitchMotorPower()));
		newSettings->setPitchMotorDirectionChanged(compare(newSettings->getPitchMotorDirection(),oldSettings->getPitchMotorDirection()));
		newSettings->setSwapXYAxisChanged(compare(newSettings->getSwapXYAxis(),oldSettings->getSwapXYAxis()));
		newSettings->setReverseXAxisChanged(compare(newSettings->getReverseXAxis(),oldSettings->getReverseXAxis()));
		newSettings->setReverseYAxisChanged(compare(newSettings->getReverseYAxis(),oldSettings->getReverseYAxis()));
		newSettings->setReverseZAxisChanged(compare(newSettings->getReverseZAxis(),oldSettings->getReverseZAxis()));
	}

	void updateGUI(Settings ^ s){
		//Motor Dir Changed
		if(s->getRollMotorDirectionChanged())
			this->groupBoxRollMotorDirection->BackColor = CHANGEDGREEN;
		else
			this->groupBoxRollMotorDirection->BackColor = SystemColors::Control;

		if(s->getPitchMotorDirectionChanged())
			this->groupBoxPitchMotorDirection->BackColor = CHANGEDGREEN;
		else
			this->groupBoxPitchMotorDirection->BackColor = SystemColors::Control;

		//Motor Power Changed
		if(s->getPitchMotorPowerChanged())
			this->textBoxPitchMotorPower->BackColor = CHANGEDGREEN;
		else
			this->textBoxPitchMotorPower->BackColor = Color::White;

		if(s->getRollMotorPowerChanged())
			this->textBoxRollMotorPower->BackColor = CHANGEDGREEN;
		else
			this->textBoxRollMotorPower->BackColor = Color::White;

		//Motor Select Changed 

		if(s->getMotorSelectChanged()){
			this->groupBoxPitchMotorSelect->BackColor = CHANGEDGREEN;
			this->groupBoxRollMotorSelect->BackColor = CHANGEDGREEN;
		}
		else{
			this->groupBoxPitchMotorSelect->BackColor = SystemColors::Control;
			this->groupBoxRollMotorSelect->BackColor = SystemColors::Control;
		}

		//PIDs changed

		if(s->getPitchPIDChanged(0))
			this->pPBox->BackColor = CHANGEDGREEN;
		else
			this->pPBox->BackColor = Color::White;

		if(s->getPitchPIDChanged(1))
			this->pIBox->BackColor = CHANGEDGREEN;
		else
			this->pIBox->BackColor = Color::White;

		if(s->getPitchPIDChanged(2))
			this->pDBox->BackColor = CHANGEDGREEN;
		else
			this->pDBox->BackColor = Color::White;

		if(s->getRollPIDChanged(0))
			this->rPBox->BackColor = CHANGEDGREEN;
		else
			this->rPBox->BackColor = Color::White;

		if(s->getRollPIDChanged(1))
			this->rIBox->BackColor = CHANGEDGREEN;
		else
			this->rIBox->BackColor = Color::White;

		if(s->getRollPIDChanged(2))
			this->rDBox->BackColor = CHANGEDGREEN;
		else
			this->rDBox->BackColor = Color::White;

		// Sensor Orientation Changed

		if(s->getSwapXYAxisChanged())
			this->checkBoxSwapXY->BackColor = CHANGEDGREEN;
		else
			this->checkBoxSwapXY->BackColor = SystemColors::Control;

		if(s->getReverseXAxisChanged())
			this->checkBoxReverseX->BackColor = CHANGEDGREEN;
		else
			this->checkBoxReverseX->BackColor = SystemColors::Control;

		if(s->getReverseYAxisChanged())
			this->checkBoxReverseY->BackColor = CHANGEDGREEN;
		else
			this->checkBoxReverseY->BackColor = SystemColors::Control;
		
		if(s->getReverseZAxisChanged())
			this->checkBoxReverseZ->BackColor = CHANGEDGREEN;
		else
			this->checkBoxReverseZ->BackColor = SystemColors::Control;

		highlightInvalid();
		this->toolStripStatusLabel1->Text = this->SerialCom->getComError();
		if(this->SerialCom->getStatus())
			this->progressBar1->Value = 100;
		else
			this->progressBar1->Value = 0;
	}

	void highlightInvalid(){
		double temp = 0;
		Double::TryParse(this->pPBox->Text,temp);
		if(temp > PMAX || temp < PMIN)
			this->pPBox->BackColor = ERRORRED;
		Double::TryParse(this->pIBox->Text,temp);
		if(temp > IMAX || temp < IMIN)
			this->pIBox->BackColor = ERRORRED;
		Double::TryParse(this->pDBox->Text,temp);
		if(temp > DMAX || temp < DMIN)
			this->pDBox->BackColor = ERRORRED;

		Double::TryParse(this->rPBox->Text,temp);
		if(temp > PMAX || temp < PMIN)
			this->rPBox->BackColor = ERRORRED;
		Double::TryParse(this->rIBox->Text,temp);
		if(temp > IMAX || temp < IMIN)
			this->rIBox->BackColor = ERRORRED;
		Double::TryParse(this->rDBox->Text,temp);
		if(temp > DMAX || temp < DMIN)
			this->rDBox->BackColor = ERRORRED;

		
	}

	void resetGui(){

		if(Form1::InvokeRequired){
			Invoke(gcnew MethodInvoker(this,&Form1::resetGui));
			return;
		}

		this->radioPitchMotor1->Checked = currentSettings->getMotorSelect();
		this->radioPitchMotor2->Checked = !(currentSettings->getMotorSelect());
		this->radioRollMotor1->Checked = radioPitchMotor2->Checked;
		this->radioRollMotor2->Checked = radioPitchMotor1->Checked;

		this->radioPitchForward->Checked=referenceSettings->getPitchMotorDirection();
		this->radioRollForward->Checked=referenceSettings->getRollMotorDirection();

		this->trackBarPitchMotorPower->Value = currentSettings->getPitchMotorPower();
		this->textBoxPitchMotorPower->Text = Convert::ToString(currentSettings->getPitchMotorPower());

		this->trackBarRollMotorPower->Value = currentSettings->getRollMotorPower();
		this->textBoxRollMotorPower->Text = Convert::ToString(currentSettings->getRollMotorPower());

		this->pPBox->Text = Convert::ToString(currentSettings->getPitchPID(0));
		this->pIBox->Text = Convert::ToString(currentSettings->getPitchPID(1));
		this->pDBox->Text = Convert::ToString(currentSettings->getPitchPID(2));

		this->rPBox->Text = Convert::ToString(currentSettings->getRollPID(0));
		this->rIBox->Text = Convert::ToString(currentSettings->getRollPID(1));
		this->rDBox->Text = Convert::ToString(currentSettings->getRollPID(2));

		this->checkBoxSwapXY->Checked = currentSettings->getSwapXYAxis();
		this->checkBoxReverseX->Checked = currentSettings->getReverseXAxis();
		this->checkBoxReverseY->Checked = currentSettings->getReverseYAxis();
		this->checkBoxReverseZ->Checked = currentSettings->getReverseZAxis();

		this->comboBoxComPort->Items->Clear();
        this->comboBoxComPort->Items->AddRange(this->SerialCom->findPorts());
	}

	void updatePitchRollGui(){
		if(Form1::InvokeRequired){
			Invoke(gcnew MethodInvoker(this,&Form1::updatePitchRollGui));
			return;
		}
		labelPitchAngle->Text = Convert::ToString(pitchDisplay);
		labelRollAngle->Text = Convert::ToString(rollDisplay);
	}
	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(Form1::typeid));
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->labelRollAngle = (gcnew System::Windows::Forms::Label());
			this->labelPitchAngle = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->checkBoxReverseY = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxReverseX = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxReverseZ = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxSwapXY = (gcnew System::Windows::Forms::CheckBox());
			this->bnCalAcc = (gcnew System::Windows::Forms::Button());
			this->bnStopReadingAngle = (gcnew System::Windows::Forms::Button());
			this->bnStartReadingAngle = (gcnew System::Windows::Forms::Button());
			this->bnCalGyro = (gcnew System::Windows::Forms::Button());
			this->statusStrip1 = (gcnew System::Windows::Forms::StatusStrip());
			this->toolStripStatusLabel1 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->comErrors = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->progressBar1 = (gcnew System::Windows::Forms::ProgressBar());
			this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
			this->radioButton1 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton2 = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox8 = (gcnew System::Windows::Forms::GroupBox());
			this->radioButton5 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButton6 = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->textBox4 = (gcnew System::Windows::Forms::TextBox());
			this->groupBox11 = (gcnew System::Windows::Forms::GroupBox());
			this->textBox5 = (gcnew System::Windows::Forms::TextBox());
			this->trackBar2 = (gcnew System::Windows::Forms::TrackBar());
			this->closeButton = (gcnew System::Windows::Forms::Button());
			this->comboBoxComPort = (gcnew System::Windows::Forms::ComboBox());
			this->initButton = (gcnew System::Windows::Forms::Button());
			this->comboBoxBaud = (gcnew System::Windows::Forms::ComboBox());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->pMotorGroup = (gcnew System::Windows::Forms::GroupBox());
			this->groupBoxPitchMotorDirection = (gcnew System::Windows::Forms::GroupBox());
			this->radioPitchReverse = (gcnew System::Windows::Forms::RadioButton());
			this->radioPitchForward = (gcnew System::Windows::Forms::RadioButton());
			this->groupBoxPitchMotorSelect = (gcnew System::Windows::Forms::GroupBox());
			this->radioPitchMotor2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioPitchMotor1 = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
			this->pDBox = (gcnew System::Windows::Forms::TextBox());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->pIBox = (gcnew System::Windows::Forms::TextBox());
			this->pPBox = (gcnew System::Windows::Forms::TextBox());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->shapeContainer2 = (gcnew Microsoft::VisualBasic::PowerPacks::ShapeContainer());
			this->textBoxPitchMotorPower = (gcnew System::Windows::Forms::TextBox());
			this->trackBarPitchMotorPower = (gcnew System::Windows::Forms::TrackBar());
			this->title = (gcnew System::Windows::Forms::Label());
			this->groupBox12 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBoxRollMotorDirection = (gcnew System::Windows::Forms::GroupBox());
			this->radioRollReverse = (gcnew System::Windows::Forms::RadioButton());
			this->radioRollForward = (gcnew System::Windows::Forms::RadioButton());
			this->groupBoxRollMotorSelect = (gcnew System::Windows::Forms::GroupBox());
			this->radioRollMotor2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioRollMotor1 = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox15 = (gcnew System::Windows::Forms::GroupBox());
			this->rDBox = (gcnew System::Windows::Forms::TextBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->rIBox = (gcnew System::Windows::Forms::TextBox());
			this->rPBox = (gcnew System::Windows::Forms::TextBox());
			this->groupBox16 = (gcnew System::Windows::Forms::GroupBox());
			this->textBoxRollMotorPower = (gcnew System::Windows::Forms::TextBox());
			this->trackBarRollMotorPower = (gcnew System::Windows::Forms::TrackBar());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->buttonReset = (gcnew System::Windows::Forms::Button());
			this->buttonLoad = (gcnew System::Windows::Forms::Button());
			this->buttonSave = (gcnew System::Windows::Forms::Button());
			this->buttonDefault = (gcnew System::Windows::Forms::Button());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->bindingSource1 = (gcnew System::Windows::Forms::BindingSource(this->components));
			this->rectangleShape1 = (gcnew Microsoft::VisualBasic::PowerPacks::RectangleShape());
			this->shapeContainer3 = (gcnew Microsoft::VisualBasic::PowerPacks::ShapeContainer());
			this->labelSetAngleRoll = (gcnew System::Windows::Forms::Label());
			this->labelSetAnglePitch = (gcnew System::Windows::Forms::Label());
			this->groupBox4->SuspendLayout();
			this->statusStrip1->SuspendLayout();
			this->groupBox1->SuspendLayout();
			this->groupBox6->SuspendLayout();
			this->groupBox7->SuspendLayout();
			this->groupBox8->SuspendLayout();
			this->groupBox9->SuspendLayout();
			this->groupBox11->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->BeginInit();
			this->pMotorGroup->SuspendLayout();
			this->groupBoxPitchMotorDirection->SuspendLayout();
			this->groupBoxPitchMotorSelect->SuspendLayout();
			this->groupBox5->SuspendLayout();
			this->groupBox3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBarPitchMotorPower))->BeginInit();
			this->groupBox12->SuspendLayout();
			this->groupBoxRollMotorDirection->SuspendLayout();
			this->groupBoxRollMotorSelect->SuspendLayout();
			this->groupBox15->SuspendLayout();
			this->groupBox16->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBarRollMotorPower))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->bindingSource1))->BeginInit();
			this->SuspendLayout();
			// 
			// groupBox4
			// 
			this->groupBox4->Controls->Add(this->labelRollAngle);
			this->groupBox4->Controls->Add(this->labelPitchAngle);
			this->groupBox4->Controls->Add(this->label11);
			this->groupBox4->Controls->Add(this->label10);
			this->groupBox4->Controls->Add(this->checkBoxReverseY);
			this->groupBox4->Controls->Add(this->checkBoxReverseX);
			this->groupBox4->Controls->Add(this->checkBoxReverseZ);
			this->groupBox4->Controls->Add(this->checkBoxSwapXY);
			this->groupBox4->Controls->Add(this->bnCalAcc);
			this->groupBox4->Controls->Add(this->bnStopReadingAngle);
			this->groupBox4->Controls->Add(this->bnStartReadingAngle);
			this->groupBox4->Controls->Add(this->bnCalGyro);
			this->groupBox4->Location = System::Drawing::Point(27, 236);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(200, 284);
			this->groupBox4->TabIndex = 28;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"IMU Control";
			// 
			// labelRollAngle
			// 
			this->labelRollAngle->AutoSize = true;
			this->labelRollAngle->Location = System::Drawing::Point(94, 247);
			this->labelRollAngle->Name = L"labelRollAngle";
			this->labelRollAngle->Size = System::Drawing::Size(28, 13);
			this->labelRollAngle->TabIndex = 25;
			this->labelRollAngle->Text = L"0.00";
			this->labelRollAngle->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// labelPitchAngle
			// 
			this->labelPitchAngle->AutoSize = true;
			this->labelPitchAngle->Location = System::Drawing::Point(9, 247);
			this->labelPitchAngle->Name = L"labelPitchAngle";
			this->labelPitchAngle->Size = System::Drawing::Size(28, 13);
			this->labelPitchAngle->TabIndex = 25;
			this->labelPitchAngle->Text = L"0.00";
			this->labelPitchAngle->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(94, 230);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(25, 13);
			this->label11->TabIndex = 24;
			this->label11->Text = L"Roll";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(6, 230);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(31, 13);
			this->label10->TabIndex = 24;
			this->label10->Text = L"Pitch";
			// 
			// checkBoxReverseY
			// 
			this->checkBoxReverseY->AutoSize = true;
			this->checkBoxReverseY->Location = System::Drawing::Point(6, 123);
			this->checkBoxReverseY->Name = L"checkBoxReverseY";
			this->checkBoxReverseY->Size = System::Drawing::Size(97, 17);
			this->checkBoxReverseY->TabIndex = 23;
			this->checkBoxReverseY->Text = L"Reverse Y axis";
			this->checkBoxReverseY->UseVisualStyleBackColor = true;
			this->checkBoxReverseY->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxReverseY_CheckedChanged);
			// 
			// checkBoxReverseX
			// 
			this->checkBoxReverseX->AutoSize = true;
			this->checkBoxReverseX->Location = System::Drawing::Point(6, 100);
			this->checkBoxReverseX->Name = L"checkBoxReverseX";
			this->checkBoxReverseX->Size = System::Drawing::Size(97, 17);
			this->checkBoxReverseX->TabIndex = 23;
			this->checkBoxReverseX->Text = L"Reverse X axis";
			this->checkBoxReverseX->UseVisualStyleBackColor = true;
			this->checkBoxReverseX->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxReverseX_CheckedChanged);
			// 
			// checkBoxReverseZ
			// 
			this->checkBoxReverseZ->AutoSize = true;
			this->checkBoxReverseZ->Location = System::Drawing::Point(6, 146);
			this->checkBoxReverseZ->Name = L"checkBoxReverseZ";
			this->checkBoxReverseZ->Size = System::Drawing::Size(97, 17);
			this->checkBoxReverseZ->TabIndex = 23;
			this->checkBoxReverseZ->Text = L"Reverse Z axis";
			this->checkBoxReverseZ->UseVisualStyleBackColor = true;
			this->checkBoxReverseZ->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxReverseZ_CheckedChanged);
			// 
			// checkBoxSwapXY
			// 
			this->checkBoxSwapXY->AutoSize = true;
			this->checkBoxSwapXY->BackColor = System::Drawing::SystemColors::Control;
			this->checkBoxSwapXY->Location = System::Drawing::Point(6, 77);
			this->checkBoxSwapXY->Name = L"checkBoxSwapXY";
			this->checkBoxSwapXY->Size = System::Drawing::Size(97, 17);
			this->checkBoxSwapXY->TabIndex = 22;
			this->checkBoxSwapXY->Text = L"Swap X/Y Axis";
			this->checkBoxSwapXY->UseVisualStyleBackColor = false;
			this->checkBoxSwapXY->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxSwapXY_CheckedChanged);
			// 
			// bnCalAcc
			// 
			this->bnCalAcc->Location = System::Drawing::Point(6, 19);
			this->bnCalAcc->Name = L"bnCalAcc";
			this->bnCalAcc->Size = System::Drawing::Size(121, 23);
			this->bnCalAcc->TabIndex = 21;
			this->bnCalAcc->Text = L"Calibrate Acc";
			this->bnCalAcc->UseVisualStyleBackColor = true;
			// 
			// bnStopReadingAngle
			// 
			this->bnStopReadingAngle->Location = System::Drawing::Point(6, 198);
			this->bnStopReadingAngle->Name = L"bnStopReadingAngle";
			this->bnStopReadingAngle->Size = System::Drawing::Size(121, 23);
			this->bnStopReadingAngle->TabIndex = 21;
			this->bnStopReadingAngle->Text = L"Stop Reading";
			this->bnStopReadingAngle->UseVisualStyleBackColor = true;
			this->bnStopReadingAngle->Click += gcnew System::EventHandler(this, &Form1::bnStopReadingAngle_Click);
			// 
			// bnStartReadingAngle
			// 
			this->bnStartReadingAngle->Location = System::Drawing::Point(6, 169);
			this->bnStartReadingAngle->Name = L"bnStartReadingAngle";
			this->bnStartReadingAngle->Size = System::Drawing::Size(121, 23);
			this->bnStartReadingAngle->TabIndex = 21;
			this->bnStartReadingAngle->Text = L"Start Reading";
			this->bnStartReadingAngle->UseVisualStyleBackColor = true;
			this->bnStartReadingAngle->Click += gcnew System::EventHandler(this, &Form1::bnStartReadingAngle_Click);
			// 
			// bnCalGyro
			// 
			this->bnCalGyro->Location = System::Drawing::Point(6, 48);
			this->bnCalGyro->Name = L"bnCalGyro";
			this->bnCalGyro->Size = System::Drawing::Size(121, 23);
			this->bnCalGyro->TabIndex = 21;
			this->bnCalGyro->Text = L"Calibrate Gyro";
			this->bnCalGyro->UseVisualStyleBackColor = true;
			// 
			// statusStrip1
			// 
			this->statusStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->toolStripStatusLabel1});
			this->statusStrip1->Location = System::Drawing::Point(0, 590);
			this->statusStrip1->Name = L"statusStrip1";
			this->statusStrip1->Size = System::Drawing::Size(1033, 22);
			this->statusStrip1->TabIndex = 27;
			this->statusStrip1->Text = L"statusStrip1";
			// 
			// toolStripStatusLabel1
			// 
			this->toolStripStatusLabel1->Name = L"toolStripStatusLabel1";
			this->toolStripStatusLabel1->Size = System::Drawing::Size(112, 17);
			this->toolStripStatusLabel1->Text = L"toolStripStatusLabel";
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->comErrors);
			this->groupBox1->Controls->Add(this->label9);
			this->groupBox1->Controls->Add(this->progressBar1);
			this->groupBox1->Controls->Add(this->groupBox6);
			this->groupBox1->Controls->Add(this->closeButton);
			this->groupBox1->Controls->Add(this->comboBoxComPort);
			this->groupBox1->Controls->Add(this->initButton);
			this->groupBox1->Controls->Add(this->comboBoxBaud);
			this->groupBox1->Controls->Add(this->label8);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Location = System::Drawing::Point(27, 43);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(200, 184);
			this->groupBox1->TabIndex = 26;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Serial Com";
			// 
			// comErrors
			// 
			this->comErrors->AutoSize = true;
			this->comErrors->Location = System::Drawing::Point(7, 165);
			this->comErrors->Name = L"comErrors";
			this->comErrors->Size = System::Drawing::Size(0, 13);
			this->comErrors->TabIndex = 18;
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(133, 73);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(59, 13);
			this->label9->TabIndex = 17;
			this->label9->Text = L"Port Status";
			// 
			// progressBar1
			// 
			this->progressBar1->Location = System::Drawing::Point(6, 73);
			this->progressBar1->Name = L"progressBar1";
			this->progressBar1->Size = System::Drawing::Size(121, 21);
			this->progressBar1->TabIndex = 15;
			// 
			// groupBox6
			// 
			this->groupBox6->Controls->Add(this->groupBox7);
			this->groupBox6->Controls->Add(this->groupBox8);
			this->groupBox6->Controls->Add(this->groupBox9);
			this->groupBox6->Controls->Add(this->groupBox11);
			this->groupBox6->Location = System::Drawing::Point(206, 165);
			this->groupBox6->Name = L"groupBox6";
			this->groupBox6->Size = System::Drawing::Size(554, 152);
			this->groupBox6->TabIndex = 25;
			this->groupBox6->TabStop = false;
			this->groupBox6->Text = L"Pitch Motor";
			// 
			// groupBox7
			// 
			this->groupBox7->Controls->Add(this->radioButton1);
			this->groupBox7->Controls->Add(this->radioButton2);
			this->groupBox7->Location = System::Drawing::Point(159, 19);
			this->groupBox7->Name = L"groupBox7";
			this->groupBox7->Size = System::Drawing::Size(126, 67);
			this->groupBox7->TabIndex = 23;
			this->groupBox7->TabStop = false;
			this->groupBox7->Text = L"Motor Direction";
			// 
			// radioButton1
			// 
			this->radioButton1->AutoSize = true;
			this->radioButton1->Location = System::Drawing::Point(13, 41);
			this->radioButton1->Name = L"radioButton1";
			this->radioButton1->Size = System::Drawing::Size(65, 17);
			this->radioButton1->TabIndex = 0;
			this->radioButton1->TabStop = true;
			this->radioButton1->Text = L"Reverse";
			this->radioButton1->UseVisualStyleBackColor = true;
			// 
			// radioButton2
			// 
			this->radioButton2->AutoSize = true;
			this->radioButton2->Location = System::Drawing::Point(13, 18);
			this->radioButton2->Name = L"radioButton2";
			this->radioButton2->Size = System::Drawing::Size(63, 17);
			this->radioButton2->TabIndex = 0;
			this->radioButton2->TabStop = true;
			this->radioButton2->Text = L"Forward";
			this->radioButton2->UseVisualStyleBackColor = true;
			// 
			// groupBox8
			// 
			this->groupBox8->Controls->Add(this->radioButton5);
			this->groupBox8->Controls->Add(this->radioButton6);
			this->groupBox8->Location = System::Drawing::Point(6, 19);
			this->groupBox8->Name = L"groupBox8";
			this->groupBox8->Size = System::Drawing::Size(126, 67);
			this->groupBox8->TabIndex = 29;
			this->groupBox8->TabStop = false;
			this->groupBox8->Text = L"Motor Select";
			// 
			// radioButton5
			// 
			this->radioButton5->AutoSize = true;
			this->radioButton5->Location = System::Drawing::Point(6, 42);
			this->radioButton5->Name = L"radioButton5";
			this->radioButton5->Size = System::Drawing::Size(61, 17);
			this->radioButton5->TabIndex = 0;
			this->radioButton5->TabStop = true;
			this->radioButton5->Text = L"Motor 2";
			this->radioButton5->UseVisualStyleBackColor = true;
			// 
			// radioButton6
			// 
			this->radioButton6->AutoSize = true;
			this->radioButton6->Location = System::Drawing::Point(6, 19);
			this->radioButton6->Name = L"radioButton6";
			this->radioButton6->Size = System::Drawing::Size(61, 17);
			this->radioButton6->TabIndex = 0;
			this->radioButton6->TabStop = true;
			this->radioButton6->Text = L"Motor 1";
			this->radioButton6->UseVisualStyleBackColor = true;
			// 
			// groupBox9
			// 
			this->groupBox9->Controls->Add(this->textBox2);
			this->groupBox9->Controls->Add(this->label2);
			this->groupBox9->Controls->Add(this->label3);
			this->groupBox9->Controls->Add(this->label4);
			this->groupBox9->Controls->Add(this->textBox3);
			this->groupBox9->Controls->Add(this->textBox4);
			this->groupBox9->Location = System::Drawing::Point(6, 92);
			this->groupBox9->Name = L"groupBox9";
			this->groupBox9->Size = System::Drawing::Size(540, 54);
			this->groupBox9->TabIndex = 23;
			this->groupBox9->TabStop = false;
			this->groupBox9->Text = L"Motor PID";
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(423, 17);
			this->textBox2->MaxLength = 10;
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(111, 20);
			this->textBox2->TabIndex = 7;
			this->textBox2->Text = L"2.5";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(377, 21);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(15, 13);
			this->label2->TabIndex = 12;
			this->label2->Text = L"D";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(194, 21);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(10, 13);
			this->label3->TabIndex = 11;
			this->label3->Text = L"I";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(7, 21);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(14, 13);
			this->label4->TabIndex = 10;
			this->label4->Text = L"P";
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(235, 17);
			this->textBox3->MaxLength = 10;
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(111, 20);
			this->textBox3->TabIndex = 8;
			this->textBox3->Text = L"5.5";
			// 
			// textBox4
			// 
			this->textBox4->Location = System::Drawing::Point(52, 17);
			this->textBox4->MaxLength = 10;
			this->textBox4->Name = L"textBox4";
			this->textBox4->Size = System::Drawing::Size(111, 20);
			this->textBox4->TabIndex = 9;
			this->textBox4->Text = L"10.0";
			// 
			// groupBox11
			// 
			this->groupBox11->Controls->Add(this->textBox5);
			this->groupBox11->Controls->Add(this->trackBar2);
			this->groupBox11->Location = System::Drawing::Point(312, 19);
			this->groupBox11->Name = L"groupBox11";
			this->groupBox11->Size = System::Drawing::Size(234, 67);
			this->groupBox11->TabIndex = 25;
			this->groupBox11->TabStop = false;
			this->groupBox11->Text = L"Motor Power";
			// 
			// textBox5
			// 
			this->textBox5->Location = System::Drawing::Point(173, 20);
			this->textBox5->MaxLength = 3;
			this->textBox5->Name = L"textBox5";
			this->textBox5->Size = System::Drawing::Size(39, 20);
			this->textBox5->TabIndex = 25;
			// 
			// trackBar2
			// 
			this->trackBar2->Location = System::Drawing::Point(6, 16);
			this->trackBar2->Name = L"trackBar2";
			this->trackBar2->Size = System::Drawing::Size(165, 45);
			this->trackBar2->TabIndex = 24;
			// 
			// closeButton
			// 
			this->closeButton->Location = System::Drawing::Point(6, 129);
			this->closeButton->Name = L"closeButton";
			this->closeButton->Size = System::Drawing::Size(121, 23);
			this->closeButton->TabIndex = 16;
			this->closeButton->Text = L"Close Port";
			this->closeButton->UseVisualStyleBackColor = true;
			this->closeButton->Click += gcnew System::EventHandler(this, &Form1::closeButton_Click);
			// 
			// comboBoxComPort
			// 
			this->comboBoxComPort->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->comboBoxComPort->FormattingEnabled = true;
			this->comboBoxComPort->Location = System::Drawing::Point(6, 19);
			this->comboBoxComPort->Name = L"comboBoxComPort";
			this->comboBoxComPort->Size = System::Drawing::Size(121, 21);
			this->comboBoxComPort->TabIndex = 11;
			this->comboBoxComPort->SelectedIndexChanged += gcnew System::EventHandler(this, &Form1::comboBoxComPort_SelectedIndexChanged);
			this->comboBoxComPort->Click += gcnew System::EventHandler(this, &Form1::comboBoxComPort_Clicked);
			// 
			// initButton
			// 
			this->initButton->Location = System::Drawing::Point(6, 100);
			this->initButton->Name = L"initButton";
			this->initButton->Size = System::Drawing::Size(121, 23);
			this->initButton->TabIndex = 15;
			this->initButton->Text = L"Open Port";
			this->initButton->UseVisualStyleBackColor = true;
			this->initButton->Click += gcnew System::EventHandler(this, &Form1::initButton_Click);
			// 
			// comboBoxBaud
			// 
			this->comboBoxBaud->AllowDrop = true;
			this->comboBoxBaud->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->comboBoxBaud->FormattingEnabled = true;
			this->comboBoxBaud->Items->AddRange(gcnew cli::array< System::Object^  >(11) {L"300", L"1200", L"2400", L"4800", L"9600", 
				L"14400", L"19200", L"28800", L"38400", L"57600", L"115200"});
			this->comboBoxBaud->Location = System::Drawing::Point(6, 46);
			this->comboBoxBaud->Name = L"comboBoxBaud";
			this->comboBoxBaud->Size = System::Drawing::Size(121, 21);
			this->comboBoxBaud->TabIndex = 12;
			this->comboBoxBaud->SelectedIndexChanged += gcnew System::EventHandler(this, &Form1::comboBoxBaud_SelectedIndexChanged);
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(133, 49);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(58, 13);
			this->label8->TabIndex = 14;
			this->label8->Text = L"Baud Rate";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(133, 22);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(50, 13);
			this->label1->TabIndex = 13;
			this->label1->Text = L"Com Port";
			// 
			// pMotorGroup
			// 
			this->pMotorGroup->Controls->Add(this->groupBoxPitchMotorDirection);
			this->pMotorGroup->Controls->Add(this->groupBoxPitchMotorSelect);
			this->pMotorGroup->Controls->Add(this->groupBox5);
			this->pMotorGroup->Controls->Add(this->groupBox3);
			this->pMotorGroup->Location = System::Drawing::Point(233, 43);
			this->pMotorGroup->Name = L"pMotorGroup";
			this->pMotorGroup->Size = System::Drawing::Size(554, 152);
			this->pMotorGroup->TabIndex = 25;
			this->pMotorGroup->TabStop = false;
			this->pMotorGroup->Text = L"Pitch Motor";
			// 
			// groupBoxPitchMotorDirection
			// 
			this->groupBoxPitchMotorDirection->BackColor = System::Drawing::SystemColors::Control;
			this->groupBoxPitchMotorDirection->BackgroundImageLayout = System::Windows::Forms::ImageLayout::None;
			this->groupBoxPitchMotorDirection->Controls->Add(this->radioPitchReverse);
			this->groupBoxPitchMotorDirection->Controls->Add(this->radioPitchForward);
			this->groupBoxPitchMotorDirection->ForeColor = System::Drawing::Color::Black;
			this->groupBoxPitchMotorDirection->Location = System::Drawing::Point(159, 19);
			this->groupBoxPitchMotorDirection->Name = L"groupBoxPitchMotorDirection";
			this->groupBoxPitchMotorDirection->Size = System::Drawing::Size(126, 67);
			this->groupBoxPitchMotorDirection->TabIndex = 23;
			this->groupBoxPitchMotorDirection->TabStop = false;
			this->groupBoxPitchMotorDirection->Text = L"Motor Direction";
			// 
			// radioPitchReverse
			// 
			this->radioPitchReverse->AutoSize = true;
			this->radioPitchReverse->Location = System::Drawing::Point(13, 41);
			this->radioPitchReverse->Name = L"radioPitchReverse";
			this->radioPitchReverse->Size = System::Drawing::Size(65, 17);
			this->radioPitchReverse->TabIndex = 0;
			this->radioPitchReverse->TabStop = true;
			this->radioPitchReverse->Text = L"Reverse";
			this->radioPitchReverse->UseVisualStyleBackColor = true;
			this->radioPitchReverse->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioPitchReverse_CheckedChanged);
			// 
			// radioPitchForward
			// 
			this->radioPitchForward->AutoSize = true;
			this->radioPitchForward->Location = System::Drawing::Point(13, 18);
			this->radioPitchForward->Name = L"radioPitchForward";
			this->radioPitchForward->Size = System::Drawing::Size(63, 17);
			this->radioPitchForward->TabIndex = 0;
			this->radioPitchForward->TabStop = true;
			this->radioPitchForward->Text = L"Forward";
			this->radioPitchForward->UseVisualStyleBackColor = true;
			this->radioPitchForward->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioPitchForward_CheckedChanged);
			// 
			// groupBoxPitchMotorSelect
			// 
			this->groupBoxPitchMotorSelect->Controls->Add(this->radioPitchMotor2);
			this->groupBoxPitchMotorSelect->Controls->Add(this->radioPitchMotor1);
			this->groupBoxPitchMotorSelect->Location = System::Drawing::Point(6, 19);
			this->groupBoxPitchMotorSelect->Name = L"groupBoxPitchMotorSelect";
			this->groupBoxPitchMotorSelect->Size = System::Drawing::Size(126, 67);
			this->groupBoxPitchMotorSelect->TabIndex = 29;
			this->groupBoxPitchMotorSelect->TabStop = false;
			this->groupBoxPitchMotorSelect->Text = L"Motor Select";
			// 
			// radioPitchMotor2
			// 
			this->radioPitchMotor2->AutoSize = true;
			this->radioPitchMotor2->Location = System::Drawing::Point(6, 42);
			this->radioPitchMotor2->Name = L"radioPitchMotor2";
			this->radioPitchMotor2->Size = System::Drawing::Size(61, 17);
			this->radioPitchMotor2->TabIndex = 0;
			this->radioPitchMotor2->TabStop = true;
			this->radioPitchMotor2->Text = L"Motor 2";
			this->radioPitchMotor2->UseVisualStyleBackColor = true;
			this->radioPitchMotor2->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioPitchMotor2_CheckedChanged);
			// 
			// radioPitchMotor1
			// 
			this->radioPitchMotor1->AutoSize = true;
			this->radioPitchMotor1->Location = System::Drawing::Point(6, 19);
			this->radioPitchMotor1->Name = L"radioPitchMotor1";
			this->radioPitchMotor1->Size = System::Drawing::Size(61, 17);
			this->radioPitchMotor1->TabIndex = 0;
			this->radioPitchMotor1->TabStop = true;
			this->radioPitchMotor1->Text = L"Motor 1";
			this->radioPitchMotor1->UseVisualStyleBackColor = true;
			this->radioPitchMotor1->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioPitchMotor1_CheckedChanged);
			// 
			// groupBox5
			// 
			this->groupBox5->Controls->Add(this->pDBox);
			this->groupBox5->Controls->Add(this->label7);
			this->groupBox5->Controls->Add(this->label6);
			this->groupBox5->Controls->Add(this->label5);
			this->groupBox5->Controls->Add(this->pIBox);
			this->groupBox5->Controls->Add(this->pPBox);
			this->groupBox5->Location = System::Drawing::Point(6, 92);
			this->groupBox5->Name = L"groupBox5";
			this->groupBox5->Size = System::Drawing::Size(540, 54);
			this->groupBox5->TabIndex = 23;
			this->groupBox5->TabStop = false;
			this->groupBox5->Text = L"Motor PID";
			// 
			// pDBox
			// 
			this->pDBox->Location = System::Drawing::Point(423, 17);
			this->pDBox->MaxLength = 10;
			this->pDBox->Name = L"pDBox";
			this->pDBox->Size = System::Drawing::Size(111, 20);
			this->pDBox->TabIndex = 7;
			this->pDBox->Text = L"2.5";
			this->pDBox->TextChanged += gcnew System::EventHandler(this, &Form1::pDBox_TextChanged);
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(377, 21);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(15, 13);
			this->label7->TabIndex = 12;
			this->label7->Text = L"D";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(194, 21);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(10, 13);
			this->label6->TabIndex = 11;
			this->label6->Text = L"I";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(7, 21);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(14, 13);
			this->label5->TabIndex = 10;
			this->label5->Text = L"P";
			// 
			// pIBox
			// 
			this->pIBox->Location = System::Drawing::Point(235, 17);
			this->pIBox->MaxLength = 10;
			this->pIBox->Name = L"pIBox";
			this->pIBox->Size = System::Drawing::Size(111, 20);
			this->pIBox->TabIndex = 8;
			this->pIBox->Text = L"5.5";
			this->pIBox->TextChanged += gcnew System::EventHandler(this, &Form1::pIBox_TextChanged);
			// 
			// pPBox
			// 
			this->pPBox->Location = System::Drawing::Point(52, 17);
			this->pPBox->MaxLength = 10;
			this->pPBox->Name = L"pPBox";
			this->pPBox->Size = System::Drawing::Size(111, 20);
			this->pPBox->TabIndex = 9;
			this->pPBox->Text = L"10.0";
			this->pPBox->TextChanged += gcnew System::EventHandler(this, &Form1::pPBox_TextChanged);
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->shapeContainer2);
			this->groupBox3->Controls->Add(this->textBoxPitchMotorPower);
			this->groupBox3->Controls->Add(this->trackBarPitchMotorPower);
			this->groupBox3->Location = System::Drawing::Point(312, 19);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(234, 67);
			this->groupBox3->TabIndex = 25;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Motor Power";
			// 
			// shapeContainer2
			// 
			this->shapeContainer2->Location = System::Drawing::Point(3, 16);
			this->shapeContainer2->Margin = System::Windows::Forms::Padding(0);
			this->shapeContainer2->Name = L"shapeContainer2";
			this->shapeContainer2->Size = System::Drawing::Size(228, 48);
			this->shapeContainer2->TabIndex = 26;
			this->shapeContainer2->TabStop = false;
			// 
			// textBoxPitchMotorPower
			// 
			this->textBoxPitchMotorPower->Location = System::Drawing::Point(173, 20);
			this->textBoxPitchMotorPower->MaxLength = 3;
			this->textBoxPitchMotorPower->Name = L"textBoxPitchMotorPower";
			this->textBoxPitchMotorPower->Size = System::Drawing::Size(39, 20);
			this->textBoxPitchMotorPower->TabIndex = 25;
			this->textBoxPitchMotorPower->TextChanged += gcnew System::EventHandler(this, &Form1::textBoxPitchMotorPower_TextChanged);
			// 
			// trackBarPitchMotorPower
			// 
			this->trackBarPitchMotorPower->Location = System::Drawing::Point(6, 16);
			this->trackBarPitchMotorPower->Maximum = 255;
			this->trackBarPitchMotorPower->Name = L"trackBarPitchMotorPower";
			this->trackBarPitchMotorPower->Size = System::Drawing::Size(165, 45);
			this->trackBarPitchMotorPower->TabIndex = 1;
			this->trackBarPitchMotorPower->TickFrequency = 16;
			this->trackBarPitchMotorPower->Scroll += gcnew System::EventHandler(this, &Form1::trackBarPitchMotorPower_Scroll);
			// 
			// title
			// 
			this->title->AutoSize = true;
			this->title->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->title->Location = System::Drawing::Point(23, 5);
			this->title->Name = L"title";
			this->title->Size = System::Drawing::Size(205, 24);
			this->title->TabIndex = 23;
			this->title->Text = L"Tuska UAV Gimbal GUI";
			// 
			// groupBox12
			// 
			this->groupBox12->Controls->Add(this->groupBoxRollMotorDirection);
			this->groupBox12->Controls->Add(this->groupBoxRollMotorSelect);
			this->groupBox12->Controls->Add(this->groupBox15);
			this->groupBox12->Controls->Add(this->groupBox16);
			this->groupBox12->Location = System::Drawing::Point(233, 208);
			this->groupBox12->Name = L"groupBox12";
			this->groupBox12->Size = System::Drawing::Size(554, 152);
			this->groupBox12->TabIndex = 25;
			this->groupBox12->TabStop = false;
			this->groupBox12->Text = L"Roll Motor";
			// 
			// groupBoxRollMotorDirection
			// 
			this->groupBoxRollMotorDirection->Controls->Add(this->radioRollReverse);
			this->groupBoxRollMotorDirection->Controls->Add(this->radioRollForward);
			this->groupBoxRollMotorDirection->Location = System::Drawing::Point(159, 19);
			this->groupBoxRollMotorDirection->Name = L"groupBoxRollMotorDirection";
			this->groupBoxRollMotorDirection->Size = System::Drawing::Size(126, 67);
			this->groupBoxRollMotorDirection->TabIndex = 23;
			this->groupBoxRollMotorDirection->TabStop = false;
			this->groupBoxRollMotorDirection->Text = L"Motor Direction";
			// 
			// radioRollReverse
			// 
			this->radioRollReverse->AutoSize = true;
			this->radioRollReverse->Location = System::Drawing::Point(13, 41);
			this->radioRollReverse->Name = L"radioRollReverse";
			this->radioRollReverse->Size = System::Drawing::Size(65, 17);
			this->radioRollReverse->TabIndex = 0;
			this->radioRollReverse->TabStop = true;
			this->radioRollReverse->Text = L"Reverse";
			this->radioRollReverse->UseVisualStyleBackColor = true;
			this->radioRollReverse->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioRollReverse_CheckedChanged);
			// 
			// radioRollForward
			// 
			this->radioRollForward->AutoSize = true;
			this->radioRollForward->Location = System::Drawing::Point(13, 18);
			this->radioRollForward->Name = L"radioRollForward";
			this->radioRollForward->Size = System::Drawing::Size(63, 17);
			this->radioRollForward->TabIndex = 0;
			this->radioRollForward->TabStop = true;
			this->radioRollForward->Text = L"Forward";
			this->radioRollForward->UseVisualStyleBackColor = true;
			this->radioRollForward->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioRollForward_CheckedChanged);
			// 
			// groupBoxRollMotorSelect
			// 
			this->groupBoxRollMotorSelect->Controls->Add(this->radioRollMotor2);
			this->groupBoxRollMotorSelect->Controls->Add(this->radioRollMotor1);
			this->groupBoxRollMotorSelect->Location = System::Drawing::Point(6, 19);
			this->groupBoxRollMotorSelect->Name = L"groupBoxRollMotorSelect";
			this->groupBoxRollMotorSelect->Size = System::Drawing::Size(126, 67);
			this->groupBoxRollMotorSelect->TabIndex = 29;
			this->groupBoxRollMotorSelect->TabStop = false;
			this->groupBoxRollMotorSelect->Text = L"Motor Select";
			// 
			// radioRollMotor2
			// 
			this->radioRollMotor2->AutoSize = true;
			this->radioRollMotor2->Location = System::Drawing::Point(6, 42);
			this->radioRollMotor2->Name = L"radioRollMotor2";
			this->radioRollMotor2->Size = System::Drawing::Size(61, 17);
			this->radioRollMotor2->TabIndex = 0;
			this->radioRollMotor2->TabStop = true;
			this->radioRollMotor2->Text = L"Motor 2";
			this->radioRollMotor2->UseVisualStyleBackColor = true;
			this->radioRollMotor2->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioRollMotor2_CheckedChanged);
			// 
			// radioRollMotor1
			// 
			this->radioRollMotor1->AutoSize = true;
			this->radioRollMotor1->Location = System::Drawing::Point(6, 19);
			this->radioRollMotor1->Name = L"radioRollMotor1";
			this->radioRollMotor1->Size = System::Drawing::Size(61, 17);
			this->radioRollMotor1->TabIndex = 0;
			this->radioRollMotor1->TabStop = true;
			this->radioRollMotor1->Text = L"Motor 1";
			this->radioRollMotor1->UseVisualStyleBackColor = true;
			this->radioRollMotor1->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioRollMotor1_CheckedChanged);
			// 
			// groupBox15
			// 
			this->groupBox15->Controls->Add(this->rDBox);
			this->groupBox15->Controls->Add(this->label14);
			this->groupBox15->Controls->Add(this->label15);
			this->groupBox15->Controls->Add(this->label16);
			this->groupBox15->Controls->Add(this->rIBox);
			this->groupBox15->Controls->Add(this->rPBox);
			this->groupBox15->Location = System::Drawing::Point(6, 92);
			this->groupBox15->Name = L"groupBox15";
			this->groupBox15->Size = System::Drawing::Size(540, 54);
			this->groupBox15->TabIndex = 23;
			this->groupBox15->TabStop = false;
			this->groupBox15->Text = L"Motor PID";
			// 
			// rDBox
			// 
			this->rDBox->Location = System::Drawing::Point(423, 17);
			this->rDBox->MaxLength = 10;
			this->rDBox->Name = L"rDBox";
			this->rDBox->Size = System::Drawing::Size(111, 20);
			this->rDBox->TabIndex = 7;
			this->rDBox->Text = L"2.5";
			this->rDBox->TextChanged += gcnew System::EventHandler(this, &Form1::rDBox_TextChanged);
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(377, 21);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(15, 13);
			this->label14->TabIndex = 12;
			this->label14->Text = L"D";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(194, 21);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(10, 13);
			this->label15->TabIndex = 11;
			this->label15->Text = L"I";
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(7, 21);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(14, 13);
			this->label16->TabIndex = 10;
			this->label16->Text = L"P";
			// 
			// rIBox
			// 
			this->rIBox->Location = System::Drawing::Point(235, 17);
			this->rIBox->MaxLength = 10;
			this->rIBox->Name = L"rIBox";
			this->rIBox->Size = System::Drawing::Size(111, 20);
			this->rIBox->TabIndex = 8;
			this->rIBox->Text = L"5.5";
			this->rIBox->TextChanged += gcnew System::EventHandler(this, &Form1::rIBox_TextChanged);
			// 
			// rPBox
			// 
			this->rPBox->Location = System::Drawing::Point(52, 17);
			this->rPBox->MaxLength = 10;
			this->rPBox->Name = L"rPBox";
			this->rPBox->Size = System::Drawing::Size(111, 20);
			this->rPBox->TabIndex = 9;
			this->rPBox->Text = L"10.0";
			this->rPBox->TextChanged += gcnew System::EventHandler(this, &Form1::rPBox_TextChanged);
			// 
			// groupBox16
			// 
			this->groupBox16->Controls->Add(this->textBoxRollMotorPower);
			this->groupBox16->Controls->Add(this->trackBarRollMotorPower);
			this->groupBox16->Location = System::Drawing::Point(312, 19);
			this->groupBox16->Name = L"groupBox16";
			this->groupBox16->Size = System::Drawing::Size(234, 67);
			this->groupBox16->TabIndex = 25;
			this->groupBox16->TabStop = false;
			this->groupBox16->Text = L"Motor Power";
			// 
			// textBoxRollMotorPower
			// 
			this->textBoxRollMotorPower->Location = System::Drawing::Point(173, 20);
			this->textBoxRollMotorPower->MaxLength = 3;
			this->textBoxRollMotorPower->Name = L"textBoxRollMotorPower";
			this->textBoxRollMotorPower->Size = System::Drawing::Size(39, 20);
			this->textBoxRollMotorPower->TabIndex = 25;
			this->textBoxRollMotorPower->TextChanged += gcnew System::EventHandler(this, &Form1::textBoxRollMotorPower_TextChanged);
			// 
			// trackBarRollMotorPower
			// 
			this->trackBarRollMotorPower->CausesValidation = false;
			this->trackBarRollMotorPower->Location = System::Drawing::Point(6, 16);
			this->trackBarRollMotorPower->Maximum = 255;
			this->trackBarRollMotorPower->Name = L"trackBarRollMotorPower";
			this->trackBarRollMotorPower->Size = System::Drawing::Size(165, 45);
			this->trackBarRollMotorPower->TabIndex = 24;
			this->trackBarRollMotorPower->TickFrequency = 16;
			this->trackBarRollMotorPower->ValueChanged += gcnew System::EventHandler(this, &Form1::trackBarRollMotorPowerValueChanged);
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 10;
			this->timer1->Tick += gcnew System::EventHandler(this, &Form1::timer1_Tick);
			// 
			// buttonReset
			// 
			this->buttonReset->Location = System::Drawing::Point(842, 78);
			this->buttonReset->Name = L"buttonReset";
			this->buttonReset->Size = System::Drawing::Size(121, 23);
			this->buttonReset->TabIndex = 29;
			this->buttonReset->Text = L"Reset";
			this->buttonReset->UseVisualStyleBackColor = true;
			this->buttonReset->Click += gcnew System::EventHandler(this, &Form1::buttonReset_Click);
			// 
			// buttonLoad
			// 
			this->buttonLoad->Location = System::Drawing::Point(842, 107);
			this->buttonLoad->Name = L"buttonLoad";
			this->buttonLoad->Size = System::Drawing::Size(121, 23);
			this->buttonLoad->TabIndex = 29;
			this->buttonLoad->Text = L"Load";
			this->buttonLoad->UseVisualStyleBackColor = true;
			this->buttonLoad->Click += gcnew System::EventHandler(this, &Form1::buttonLoad_Click);
			// 
			// buttonSave
			// 
			this->buttonSave->Location = System::Drawing::Point(842, 136);
			this->buttonSave->Name = L"buttonSave";
			this->buttonSave->Size = System::Drawing::Size(121, 23);
			this->buttonSave->TabIndex = 29;
			this->buttonSave->Text = L"Save";
			this->buttonSave->UseVisualStyleBackColor = true;
			this->buttonSave->Click += gcnew System::EventHandler(this, &Form1::buttonSave_Click);
			// 
			// buttonDefault
			// 
			this->buttonDefault->Location = System::Drawing::Point(842, 49);
			this->buttonDefault->Name = L"buttonDefault";
			this->buttonDefault->Size = System::Drawing::Size(121, 23);
			this->buttonDefault->TabIndex = 29;
			this->buttonDefault->Text = L"Default";
			this->buttonDefault->UseVisualStyleBackColor = true;
			this->buttonDefault->Click += gcnew System::EventHandler(this, &Form1::buttonDefault_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->ErrorImage = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"pictureBox1.ErrorImage")));
			this->pictureBox1->ImageLocation = L"./Banner1 mono.png";
			this->pictureBox1->InitialImage = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"pictureBox1.InitialImage")));
			this->pictureBox1->Location = System::Drawing::Point(807, 165);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(200, 68);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::StretchImage;
			this->pictureBox1->TabIndex = 30;
			this->pictureBox1->TabStop = false;
			this->pictureBox1->WaitOnLoad = true;
			// 
			// rectangleShape1
			// 
			this->rectangleShape1->BackColor = System::Drawing::SystemColors::Menu;
			this->rectangleShape1->BorderColor = System::Drawing::Color::Black;
			this->rectangleShape1->Location = System::Drawing::Point(239, 366);
			this->rectangleShape1->Name = L"rectangleShape1";
			this->rectangleShape1->Size = System::Drawing::Size(201, 201);
			this->rectangleShape1->Click += gcnew System::EventHandler(this, &Form1::rectangleShape1_Click);
			this->rectangleShape1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::rectangleShape1_MouseDown);
			// 
			// shapeContainer3
			// 
			this->shapeContainer3->Location = System::Drawing::Point(0, 0);
			this->shapeContainer3->Margin = System::Windows::Forms::Padding(0);
			this->shapeContainer3->Name = L"shapeContainer3";
			this->shapeContainer3->Shapes->AddRange(gcnew cli::array< Microsoft::VisualBasic::PowerPacks::Shape^  >(1) {this->rectangleShape1});
			this->shapeContainer3->Size = System::Drawing::Size(1033, 612);
			this->shapeContainer3->TabIndex = 32;
			this->shapeContainer3->TabStop = false;
			// 
			// labelSetAngleRoll
			// 
			this->labelSetAngleRoll->AutoSize = true;
			this->labelSetAngleRoll->Location = System::Drawing::Point(236, 572);
			this->labelSetAngleRoll->Name = L"labelSetAngleRoll";
			this->labelSetAngleRoll->Size = System::Drawing::Size(28, 13);
			this->labelSetAngleRoll->TabIndex = 33;
			this->labelSetAngleRoll->Text = L"0.00";
			// 
			// labelSetAnglePitch
			// 
			this->labelSetAnglePitch->AutoSize = true;
			this->labelSetAnglePitch->Location = System::Drawing::Point(412, 572);
			this->labelSetAnglePitch->Name = L"labelSetAnglePitch";
			this->labelSetAnglePitch->Size = System::Drawing::Size(28, 13);
			this->labelSetAnglePitch->TabIndex = 33;
			this->labelSetAnglePitch->Text = L"0.00";
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1033, 612);
			this->Controls->Add(this->labelSetAnglePitch);
			this->Controls->Add(this->labelSetAngleRoll);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->buttonSave);
			this->Controls->Add(this->buttonLoad);
			this->Controls->Add(this->buttonDefault);
			this->Controls->Add(this->buttonReset);
			this->Controls->Add(this->groupBox4);
			this->Controls->Add(this->statusStrip1);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->groupBox12);
			this->Controls->Add(this->pMotorGroup);
			this->Controls->Add(this->title);
			this->Controls->Add(this->shapeContainer3);
			this->Name = L"Form1";
			this->Text = L"Form1";
			this->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &Form1::Form1_Paint);
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			this->statusStrip1->ResumeLayout(false);
			this->statusStrip1->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBox6->ResumeLayout(false);
			this->groupBox7->ResumeLayout(false);
			this->groupBox7->PerformLayout();
			this->groupBox8->ResumeLayout(false);
			this->groupBox8->PerformLayout();
			this->groupBox9->ResumeLayout(false);
			this->groupBox9->PerformLayout();
			this->groupBox11->ResumeLayout(false);
			this->groupBox11->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBar2))->EndInit();
			this->pMotorGroup->ResumeLayout(false);
			this->groupBoxPitchMotorDirection->ResumeLayout(false);
			this->groupBoxPitchMotorDirection->PerformLayout();
			this->groupBoxPitchMotorSelect->ResumeLayout(false);
			this->groupBoxPitchMotorSelect->PerformLayout();
			this->groupBox5->ResumeLayout(false);
			this->groupBox5->PerformLayout();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBarPitchMotorPower))->EndInit();
			this->groupBox12->ResumeLayout(false);
			this->groupBoxRollMotorDirection->ResumeLayout(false);
			this->groupBoxRollMotorDirection->PerformLayout();
			this->groupBoxRollMotorSelect->ResumeLayout(false);
			this->groupBoxRollMotorSelect->PerformLayout();
			this->groupBox15->ResumeLayout(false);
			this->groupBox15->PerformLayout();
			this->groupBox16->ResumeLayout(false);
			this->groupBox16->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void radioPitchMotor1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
				 this->radioRollMotor2->Checked = this->radioPitchMotor1->Checked;
				 this->radioRollMotor1->Checked = !(this->radioPitchMotor1->Checked);
				 this->currentSettings->setMotorSelect(this->radioPitchMotor1->Checked);
			 }
	private: System::Void radioPitchMotor2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->radioRollMotor2->Checked = !this->radioPitchMotor2->Checked;
			 this->radioRollMotor1->Checked =  this->radioPitchMotor2->Checked;
			 this->currentSettings->setMotorSelect(!this->radioPitchMotor2->Checked);
				 
		 }

	private: System::Void radioRollMotor1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->radioPitchMotor2->Checked =   this->radioRollMotor1->Checked;
			 this->radioPitchMotor1->Checked =  !this->radioRollMotor1->Checked;
			 this->currentSettings->setMotorSelect(!this->radioRollMotor1->Checked);
		
		 }
private: System::Void radioRollMotor2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->radioPitchMotor2->Checked =   !this->radioRollMotor2->Checked;
			 this->radioPitchMotor1->Checked =   this->radioRollMotor2->Checked;
			 this->currentSettings->setMotorSelect(this->radioRollMotor2->Checked);


		 }
private: System::Void trackBarPitchMotorPower_Scroll(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setPitchMotorPower(this->trackBarPitchMotorPower->Value);
			 this->textBoxPitchMotorPower->Text = System::Convert::ToString(this->currentSettings->getPitchMotorPower());
			// Console::WriteLine("Scroll!");
			// int barSize = this->currentSettings->getPitchMotorPower()/2;
		 }
private: System::Void textBoxPitchMotorPower_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			int number = currentSettings->getPitchMotorPower();
			
			try{
				number = Int16::Parse(textBoxPitchMotorPower->Text);
			   // Console::WriteLine("Converted '{0}' to {1}.", textBoxPitchMotorPower->Text, number);
			}
			catch (FormatException ^e){
				
			}

			if(this->currentSettings->setPitchMotorPower(number)){
				textBoxPitchMotorPower->Text= Convert::ToString(currentSettings->getPitchMotorPower());
				// add warning
			}
			else{
				trackBarPitchMotorPower->Value = currentSettings->getPitchMotorPower();
				
			}
					
		}

private: System::Void textBoxRollMotorPower_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 int number = currentSettings->getRollMotorPower();
			
			try{
				number = Int16::Parse(textBoxRollMotorPower->Text);
			   // Console::WriteLine("Converted '{0}' to {1}.", textBoxPitchMotorPower->Text, number);
			}
			catch (FormatException ^e){
				
			}

			if(this->currentSettings->setRollMotorPower(number)){
				textBoxRollMotorPower->Text= Convert::ToString(currentSettings->getRollMotorPower());
				// add warning
			}
			else{
				trackBarRollMotorPower->Value = currentSettings->getRollMotorPower();
				
			}
		 }
private: System::Void trackBarRollMotorPowerValueChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setRollMotorPower(this->trackBarRollMotorPower->Value);
			 this->textBoxRollMotorPower->Text = System::Convert::ToString(this->currentSettings->getRollMotorPower());
			 //Console::WriteLine("Scroll!");

		 }
private: System::Void radioPitchForward_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			this->currentSettings->setPitchMotorDirection(1);
		 }
private: System::Void radioPitchReverse_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setPitchMotorDirection(0);
		 }
private: System::Void radioRollForward_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setRollMotorDirection(1);
		 }
private: System::Void radioRollReverse_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			this->currentSettings->setRollMotorDirection(0);
		 }

private: System::Void checkBoxSwapXY_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setSwapXYAxis(this->checkBoxSwapXY->Checked);
		 }
private: System::Void checkBoxReverseX_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setReverseXAxis(this->checkBoxReverseX->Checked);
		 }
private: System::Void checkBoxReverseY_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setReverseYAxis(this->checkBoxReverseY->Checked);
		 }
private: System::Void checkBoxReverseZ_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings->setReverseZAxis(this->checkBoxReverseZ->Checked);
		 }

private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
			 mainTick++;
			 this->checkForChanges(currentSettings,referenceSettings);
			 this->updateGUI(currentSettings);
		 }

private: System::Void serialDataRecieved(System::Object^  sender, System::EventArgs^  e) {
			// this->currentSettings->setReverseYAxis(this->checkBoxReverseY->Checked);
		 }

// PID Box Events

private: System::Void pPBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 double temp = 0;
			 if(Double::TryParse(this->pPBox->Text,temp))
				this->currentSettings->setPitchPID(0,temp);
		 }
private: System::Void pIBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 double temp = 0;
			 if(Double::TryParse(this->pIBox->Text,temp))
				  this->currentSettings->setPitchPID(1,temp);
		 }

private: System::Void pDBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 double temp = 0;
			 if(Double::TryParse(this->pDBox->Text,temp))
				this->currentSettings->setPitchPID(2,temp);
		 }

private: System::Void rPBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 double temp = 0;
			 if(Double::TryParse(this->rPBox->Text,temp))
				 this->currentSettings->setRollPID(0,temp);
		 }
private: System::Void rIBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 double temp = 0;
			 if(Double::TryParse(this->rIBox->Text,temp))
				 this->currentSettings->setRollPID(1,temp);
		 }
private: System::Void rDBox_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 double temp = 0;
			 if(Double::TryParse(this->rDBox->Text,temp))
				this->currentSettings->setRollPID(2,temp);
		 }

//Save Button
private: System::Void buttonSave_Click(System::Object^  sender, System::EventArgs^  e) {
			this->referenceSettings = gcnew Settings(currentSettings);
			this->sendPIDs();
			this->sendMotorPower();
			resetGui();
		 }

private: System::Void buttonLoad_Click(System::Object^  sender, System::EventArgs^  e) {
			this->readPIDs();
			this->readMotorPower();
		 }


private: System::Void buttonReset_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings = gcnew Settings(referenceSettings);
			 this->resetGui();
		 }
private: System::Void buttonDefault_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->currentSettings = gcnew Settings();
			 this->resetGui();
		 }

// Serial Stuff
private: System::Void comboBoxComPort_Clicked(System::Object^  sender, System::EventArgs^  e) {
			 this->comboBoxComPort->Items->Clear();
             this->comboBoxComPort->Items->AddRange(this->SerialCom->findPorts());
		 }

private: System::Void comboBoxComPort_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->SerialCom->setComPort(this->comboBoxComPort->SelectedItem->ToString());
		 }

private: System::Void initButton_Click(System::Object^  sender, System::EventArgs^  e) {
			this->SerialCom->connect();
			if(SerialCom->getStatus()){
				this->readPIDs();
				this->readMotorPower();
			}
		 }

private: System::Void closeButton_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->SerialCom->disconect();
		 }

private: System::Void comboBoxBaud_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->SerialCom->setBaudRate(Convert::ToInt32(comboBoxBaud->SelectedItem));
		 }

private: System::Void bnStartReadingAngle_Click(System::Object^  sender, System::EventArgs^  e) {
			 updateAngles = true;
			 SerialCom->sendCommand("AST");
		 }
private: System::Void bnStopReadingAngle_Click(System::Object^  sender, System::EventArgs^  e) {
			 SerialCom->sendCommand("ASP");
			 updateAngles = false;
			 pitchDisplay=0;
			 rollDisplay=0;
			 updatePitchRollGui();
		 }

private: System::Void Form1_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
			int rectangleX = rectangleShape1->Location.X;
			int rectangleY = rectangleShape1->Location.Y;
			
			int rectangleW = rectangleShape1->Width;
			int rectangleH = rectangleShape1->Height;
			int rectangleXRight = rectangleX + rectangleW;
			int rectangleYBottom = rectangleY + rectangleH;
			int rectangleCenterX = (rectangleX + (rectangleXRight))/2;
			int rectangleCenterY = (rectangleY + (rectangleYBottom))/2;

			gBackBuffer->DrawRectangle(gcnew Pen(Color::Black),rectangleX,rectangleY,rectangleW,rectangleH);
			gBackBuffer->DrawLine(gcnew Pen(Color::Black),rectangleCenterX,rectangleY,rectangleCenterX,rectangleYBottom);
			gBackBuffer->DrawLine(gcnew Pen(Color::Black),rectangleX,rectangleCenterY,rectangleXRight,rectangleCenterY);

			//rectangleShape1->dra(gBackBuffer,rectangleShape1->Bounds);
			//this->rectangleShape1->Location = System::Drawing::Point(239, 366);
			
			//this->rectangleShape1->Size = System::Drawing::Size(200, 200);
			e->Graphics->DrawImageUnscaled(backBuffer, 0, 0);
			gBackBuffer->Clear(Color::Transparent);
			 }

void processSerialCommands(){
	while(1){
		if(SerialCom->commandAvailable()){
			String^ command = SerialCom->processCommand();
			Console::WriteLine(command);
			array<String^>^ splitCommand = command->Split(' ');
			String^commandType = "";
			if(splitCommand->Length > 0){
				commandType = splitCommand[0];
			}
			if(commandType == "SPID")
				serialSetPID(splitCommand);
			else if(commandType == "SMP")
				serialSetMotorPower(splitCommand);
			else if(commandType == "PRA")
				serialUpdatePitchRoll(splitCommand);
		}

	}
}

void serialSetPID(array<String^>^ s){
	int index = 0;
	double value = 0;

	int::TryParse(s[2],index);
	double::TryParse(s[3],value);

	if(s[1]->ToLower() == "p"){
		currentSettings->setPitchPID(index,value);
		referenceSettings->setPitchPID(index,value);
		resetGui();
		updateGUI(currentSettings);
	}

	if(s[1]->ToLower() == "r"){
		resetGui();
		currentSettings->setRollPID(index,value);
		referenceSettings->setRollPID(index,value);
		updateGUI(currentSettings);
	}
}

void serialSetMotorPower(array<String^>^s){
	UInt16 value = 0;
	UInt16::TryParse(s[2],value);

	if(s[1]->ToLower() == "p"){
		currentSettings->setPitchMotorPower(value);
		referenceSettings->setPitchMotorPower(value);
		resetGui();
		updateGUI(currentSettings);

	}
	if(s[1]->ToLower() == "r"){
		resetGui();
		currentSettings->setRollMotorPower(value);
		referenceSettings->setRollMotorPower(value);
		resetGui();
		updateGUI(currentSettings);
	}
	
}

void serialUpdatePitchRoll(array<String^>^s){
	if(updateAngles){
		double pitch = 0;
		double roll = 0;
		double::TryParse(s[1],pitch);
		double::TryParse(s[2],roll);

		pitchDisplay = pitch;
		rollDisplay = roll;
	
		updatePitchRollGui();
	}
}

private: System::Void rectangleShape1_Click(System::Object^  sender, System::EventArgs^  e) {
			//Console::WriteLine(this->Location.X + this->Location.Y);
			int mouseX = rectangleShape1->PointToClient(Cursor->Position).X;
			int mouseY = rectangleShape1->PointToClient(Cursor->Position).Y;
			int mouseXdraw = MousePosition.X;
			int mouseYdraw = MousePosition.Y;
			int centerX = rectangleShape1->Width/2;
			int centerY = rectangleShape1->Height/2;
			int offsetX = mouseX - centerX;
			int offsetY = mouseY - centerY;
			Console::WriteLine(offsetX+","+offsetY); 
			//gBackBuffer->FillEllipse(gcnew SolidBrush(Color::Black),Form1::PointToClient(Cursor->Position).X-3,Form1::PointToClient(Cursor->Position).Y-3,6,6);
			drawPointer();
			//Invalidate();
			if(SerialCom->getStatus())
				sendAngleSetCommand();
		 }
private: System::Void rectangleShape1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			//Console::WriteLine(this->Location.X + this->Location.Y);
			mouseDown = gcnew Thread(gcnew ThreadStart(this, &Form1::whileMouseDown));
			mouseDown->Start();
			//SerialCom->sendCommand("SA " + Convert::ToString(-1*offsetY/3) + " " + Convert::ToString(offsetX/3));
		 }

void whileMouseDown(){
	drawPointer();
	UInt64 pTick = mainTick;
	while(Convert::ToBoolean((Convert::ToInt32(Control::MouseButtons) & 1 << 20) >> 20)){
		if(mainTick - pTick > 0){
			drawPointer();
			sendAngleSetCommand();
			pTick = mainTick;
		}
	}
	mouseDown->Join();
}

void drawPointer(){
	if(Form1::InvokeRequired){
		Invoke(gcnew MethodInvoker(this,&Form1::drawPointer));
		return;
	}

	int mouseX = rectangleShape1->PointToClient(Cursor->Position).X;
	int mouseY = rectangleShape1->PointToClient(Cursor->Position).Y;
	int mouseXdraw = MousePosition.X;
	int mouseYdraw = MousePosition.Y;
	int centerX = rectangleShape1->Width/2;
	int centerY = rectangleShape1->Height/2;
	int offsetX = mouseX - centerX;
	int offsetY = mouseY - centerY;
	if(mouseX >= 0 && mouseX <= rectangleShape1->Width && mouseY >= 0 && mouseY <= rectangleShape1->Height){ // make sure cursor does not leave bounding rectangle
	//Console::WriteLine(offsetX+","+offsetY);
	double p = -1*offsetY/3.f;
	double r = offsetX/3.f;
	if(p<= 0.5 && p>= -0.5)
		p = 0;
	if(r<= 0.5 && r>= -0.5)
		r = 0;
	this->labelSetAnglePitch->Text = String::Format("{0:0.00}", p);
	this->labelSetAngleRoll->Text = String::Format("{0:0.00}", r);	
	pitchAngleSet = p;
	rollAngleSet = r;
	if(p == 0 && r == 0)
		gBackBuffer->FillEllipse(gcnew SolidBrush(Color::Blue),rectangleShape1->Location.X+centerX-5,rectangleShape1->Location.Y+centerY-5,10,10);
	else
		gBackBuffer->FillEllipse(gcnew SolidBrush(Color::Black),Form1::PointToClient(Cursor->Position).X-3,Form1::PointToClient(Cursor->Position).Y-3,6,6);
	Invalidate();
}
}

void sendAngleSetCommand(){
	if(Form1::InvokeRequired){
		Invoke(gcnew MethodInvoker(this,&Form1::sendAngleSetCommand));
		return;
	}
	if(SerialCom->getStatus()){
			Console::WriteLine("AS " + String::Format("{0:0.00}", pitchAngleSet) + " " + String::Format("{0:0.00}", rollAngleSet));
			SerialCom->sendCommand("AS " + String::Format("{0:0.00}", pitchAngleSet) + " " + String::Format("{0:0.00}", rollAngleSet));
	}
}

void sendPIDs(){
	for(int i=0; i<3; i++){
		String ^pitchMessage = "SPID P " + Convert::ToString(i) +" " + Convert::ToString(currentSettings->getPitchPID(i));
		String ^rollMessage = "SPID R " + Convert::ToString(i) +" " + Convert::ToString(currentSettings->getRollPID(i));
		SerialCom->sendCommand(pitchMessage);
		SerialCom->sendCommand(rollMessage);
	}
}

void readPIDs(){
	for(int i=0; i<3; i++){
		String ^pitchMessage = "RPID P " + Convert::ToString(i);
		String ^rollMessage = "RPID R " + Convert::ToString(i);
		SerialCom->sendCommand(pitchMessage);
		SerialCom->sendCommand(rollMessage);
	}
}

void sendMotorPower(){
	String ^pitchMessage = "SMP P " + Convert::ToString(currentSettings->getPitchMotorPower());
	String ^rollMessage = "SMP R "  + Convert::ToString(currentSettings->getRollMotorPower());
	SerialCom->sendCommand(pitchMessage);
	SerialCom->sendCommand(rollMessage);
}

void readMotorPower(){
		String ^pitchMessage = "RMP P";
		String ^rollMessage = "RMP R";
		SerialCom->sendCommand(pitchMessage);
		SerialCom->sendCommand(rollMessage);
}


};
}