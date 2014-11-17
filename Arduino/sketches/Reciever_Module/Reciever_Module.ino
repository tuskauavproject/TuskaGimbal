#define icpPin            8         // this interrupt handler must use pin 8
#define TICKS_PER_uS      2          // number of timer ticks per microsecond
#define MAX_CHANNELS    1         // maximum number of channels we can store  
#define SYNC_GAP_LEN      (3000 * TICKS_PER_uS) // we assume a space at least 3000us is sync (note clock counts in 0.5 us ticks) 
volatile unsigned int Pulses[ MAX_CHANNELS + 1]; // array holding channel pulses width value in microseconds 
volatile uint8_t  Channel;      // number of channels detected so far in the frame (first channel is 1)
volatile uint8_t State;         // this will be one of the following states:
#define NOT_SYNCHED_state  0    // the system is not synched so the data is random
#define ACQUIRING_state  1      // one sync pulse detected but not all channels have been received 
#define READY_state     2       // synched and all channel data is valid 

void setup()                    // run once, when the sketch starts
{
 Serial.begin(115200);   
 rcSetup();
}

void loop()                     // run over and over again
{
int pulsewidth;

  // print the decoder state
  if(State == NOT_SYNCHED_state)
      Serial.println("The decoder has not detected a synch pulse ");   
  else if ( State == ACQUIRING_state)
      Serial.println("The decoder has detected one synch pulse and has started filling channel data");  
  else if( State == READY_state)
    Serial.println("The decoder is synched and the channel data is valid");  
  else
    Serial.println("Unknown decoder state, this should never happen!"); 
  

 // now print the channel pulse widths
 // they should be 0 if the state is not ready 
 for ( int i =1; i <=1; i++ ){ // print the status of the first four channels
     Serial.print("Channel ");
     Serial.print(i);
     Serial.print(" has width ");
     pulsewidth = GetChannelPulseWidth(i);
     Serial.println(pulsewidth);
 }
 delay(100); // update 10 times a second        
}