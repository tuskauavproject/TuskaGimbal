
//volatile byte stateRCInt = 0;    // a counter to see how many times the pin has changed
//volatile uint32_t lastTimeRising = 0;
//volatile uint16_t lastPulse = 0;

void initRCDecode(){
  pinMode(RCPIN1, INPUT);     //set the pin to input
  digitalWrite(RCPIN1, HIGH); //use the internal pullup resistor
  PCintPort::attachInterrupt(RCPIN1,rcINTRising,RISING); // attach a PinChange Interrupt to our pin on the rising edge
  //PCintPort::attachInterrupt(RCPIN1,rcINTFalling,FALLING); // attach a PinChange Interrupt to our pin on the rising edge
}

void rcINTRising(){
    noInterrupts();
    lastTimeRising = timer2.get_count();
    stateRCInt = 0;
    interrupts();
 }
void rcINTFalling(){
  noInterrupts();
  uint32_t newTimeFalling = timer2.get_count();
  stateRCInt = 1;
  lastPulse = (newTimeFalling - lastTimeRising) >> 3;
  interrupts();
}

void rcINTStateChange(){
  if(stateRCInt == 0){
    PCintPort::detachInterrupt(RCPIN1);
    PCintPort::attachInterrupt(RCPIN1,rcINTFalling,FALLING);
  }
  else{
    PCintPort::detachInterrupt(RCPIN1);
    PCintPort::attachInterrupt(RCPIN1,rcINTRising,RISING);
  }
  if(lastPulse >= 1000 && lastPulse <= 2000){
    rcPulse1 = rcPulse1 * (1.0f - (1.0f/100)) + lastPulse * (1.0f/100);
  } 
}

