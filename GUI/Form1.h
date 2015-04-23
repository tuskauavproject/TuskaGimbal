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

	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{

			InitializeComponent();
			referenceSettings = gcnew Settings();
			currentSettings = gcnew Settings(); //use default constructor for now
			SerialCom = gcnew Serial();
			resetGui();

			
			
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

	private: System::Windows::Forms::GroupBox^  groupBox4;
	protected: 
	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::CheckBox^  checkBoxReverseY;

	private: System::Windows::Forms::CheckBox^  checkBoxReverseX;
	private: System::Windows::Forms::CheckBox^  checkBoxReverseZ;


	private: System::Windows::Forms::CheckBox^  checkBoxSwapXY;

	private: System::Windows::Forms::Button^  bnCalAcc;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::Button^  bnStartReading;
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
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->checkBoxReverseY = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxReverseX = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxReverseZ = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxSwapXY = (gcnew System::Windows::Forms::CheckBox());
			this->bnCalAcc = (gcnew System::Windows::Forms::Button());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->bnStartReading = (gcnew System::Windows::Forms::Button());
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
			this->groupBox4->Controls->Add(this->label13);
			this->groupBox4->Controls->Add(this->label12);
			this->groupBox4->Controls->Add(this->label11);
			this->groupBox4->Controls->Add(this->label10);
			this->groupBox4->Controls->Add(this->checkBoxReverseY);
			this->groupBox4->Controls->Add(this->checkBoxReverseX);
			this->groupBox4->Controls->Add(this->checkBoxReverseZ);
			this->groupBox4->Controls->Add(this->checkBoxSwapXY);
			this->groupBox4->Controls->Add(this->bnCalAcc);
			this->groupBox4->Controls->Add(this->button1);
			this->groupBox4->Controls->Add(this->bnStartReading);
			this->groupBox4->Controls->Add(this->bnCalGyro);
			this->groupBox4->Location = System::Drawing::Point(27, 236);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(200, 284);
			this->groupBox4->TabIndex = 28;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"IMU Control";
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(94, 247);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(28, 13);
			this->label13->TabIndex = 25;
			this->label13->Text = L"0.00";
			this->label13->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(9, 247);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(28, 13);
			this->label12->TabIndex = 25;
			this->label12->Text = L"0.00";
			this->label12->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
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
			// button1
			// 
			this->button1->Location = System::Drawing::Point(6, 198);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(121, 23);
			this->button1->TabIndex = 21;
			this->button1->Text = L"Stop Reading";
			this->button1->UseVisualStyleBackColor = true;
			// 
			// bnStartReading
			// 
			this->bnStartReading->Location = System::Drawing::Point(6, 169);
			this->bnStartReading->Name = L"bnStartReading";
			this->bnStartReading->Size = System::Drawing::Size(121, 23);
			this->bnStartReading->TabIndex = 21;
			this->bnStartReading->Text = L"Start Reading";
			this->bnStartReading->UseVisualStyleBackColor = true;
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
			this->statusStrip1->Location = System::Drawing::Point(0, 560);
			this->statusStrip1->Name = L"statusStrip1";
			this->statusStrip1->Size = System::Drawing::Size(1019, 22);
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
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1019, 582);
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
			this->Name = L"Form1";
			this->Text = L"Form1";
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
//			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->trackBarRollMotorPower))->EndInit();
//			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->EndInit();
//			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->bindingSource1))->EndInit();
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
			 this->checkForChanges(currentSettings,referenceSettings);
			 this->updateGUI(currentSettings);
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
			resetGui();
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
		 }

private: System::Void closeButton_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->SerialCom->disconect();
		 }

private: System::Void comboBoxBaud_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->SerialCom->setBaudRate(Convert::ToInt32(comboBoxBaud->SelectedItem));
		 }
};
}
