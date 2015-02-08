// motor drive

void initBlController() 
{
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); //timer0 control register see datasheet
  TCCR0B = _BV(CS00);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  // enable Timer 1 interrupt
  TIMSK1 |= _BV(TOIE1);
  
  // disable arduino standard timer interrupt
  TIMSK0 &= ~_BV(TOIE1);
  
  sei();

  // Enable Timer1 Interrupt for Motor Control
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5 
}

void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint16_t maxPWM)
{
  uint16_t posStep;
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // fetch pwm from sinus table
  posStep = MotorPos & 0xff; // constrain posStep to 8 LSB 
  pwm_a = pwmSinMotor[(uint8_t)posStep];
  pwm_b = pwmSinMotor[(uint8_t)(posStep + phaseAngle1)]; //
  pwm_c = pwmSinMotor[(uint8_t)(posStep + phaseAngle2)];
 
  // power factor
  pwm_a = maxPWM * pwm_a;
  pwm_a = pwm_a >> 8; // divide by 256
  pwm_a += 128;

  pwm_b = maxPWM * pwm_b;
  pwm_b = pwm_b >> 8;
  pwm_b += 128;
  
  pwm_c = maxPWM * pwm_c;
  pwm_c = pwm_c >> 8;
  pwm_c += 128;
  
  // set motor pwm variables
  if (motorNumber == 0)
  {
    pwm_a_motor0 = (uint8_t)pwm_a;
    pwm_b_motor0 = (uint8_t)pwm_b;
    pwm_c_motor0 = (uint8_t)pwm_c;
  }
 
  if (motorNumber == 1)
  {
    pwm_a_motor1 = (uint8_t)pwm_a;
    pwm_b_motor1 = (uint8_t)pwm_b;
    pwm_c_motor1 = (uint8_t)pwm_c;
  }
}


void calcSinusArray()
{
  for(int i=0; i<N_SIN; i++)
  {
    pwmSinMotor[i] =  sin(2.0 * i / N_SIN * 3.14159265) * 127.0;
  }  
}

void initMotorStuff()
{
  cli();
  calcSinusArray();
  sei();
}

int32_t ComputePID(int32_t DTms, int32_t DTinv, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd)
{
  float error = (setPoint - in);
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTinv;
  *errorOld = error;

  out = out / 4096 / 8;
  
  return out;
  
}

ISR( TIMER1_OVF_vect )
{
  freqCounter++;  
  if(freqCounter==(CC_FACTOR/MOTORUPDATE_FREQ))
  {
    freqCounter=0;
    
    PWM_A_MOTOR0 = pwm_a_motor0;
    PWM_B_MOTOR0 = pwm_b_motor0;
    PWM_C_MOTOR0 = pwm_c_motor0;

    PWM_A_MOTOR1 = pwm_a_motor1;
    PWM_B_MOTOR1 = pwm_b_motor1;
    PWM_C_MOTOR1 = pwm_c_motor1;  
    
    // update event
    motorUpdate = true;
  }
  
  // care for standard timers every 1 ms
  if ((freqCounter & 0x01f) == 0) {
    TIMER0_isr_emulation();
  }
}