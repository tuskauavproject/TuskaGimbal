ISR(TIMER1_CAPT_vect){ 
  if(! bit_is_set(TCCR1B ,ICES1)){       // was falling edge detected ?   
      TCNT1 = 0;               // reset the counter      
      if(Channel <= MAX_CHANNELS) {
          Pulses[Channel++] = ICR1 / TICKS_PER_uS;  // store pulse length as microsoeconds
       }      
  }
  else {                          // rising  edge was detected   
       TCNT1 = 0;               // reset the counter      
       if(ICR1 >= SYNC_GAP_LEN){   // is the space between pulses big enough to be the SYNC
           Channel = 1;       // if so, reset the channel counter to 1       
             if(State == NOT_SYNCHED_state)
                 State = ACQUIRING_state;        // this is the first sync pulse, we need one more to fill the channel data array
             else if( State == ACQUIRING_state)     
                  State = READY_state;           // this is the second sync so flag that channel data is valid
       }    
  }     
  TCCR1B ^= _BV(ICES1);                 // toggle bit value to trigger on the other edge    
}

void rcSetup()
{
  pinMode(icpPin,INPUT);
 Channel = 1;             
 State = NOT_SYNCHED_state;
 TCCR1A = 0x00;         // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled 
 TCCR1B = 0x02;         // 16MHz clock with prescaler means TCNT1 increments every .5 uS (cs11 bit set
 TIMSK1 = _BV(ICIE1);   // enable input capture interrupt for timer 1
}

int GetChannelPulseWidth( uint8_t channel) {
 // this is the access function for channel data
 int result;  
 if( (State == READY_state)  && (channel > 0) && (channel <=  MAX_CHANNELS)  ) {
    cli();             //disable interrupts
    result =  Pulses[channel] ;
    sei();             // enable interrupts
 }
 else
    result = 0;        // return 0 if no valid pulse is available  

 return result; 
}