void IMU(){
  for(axis = 0; axis < 3; axis++)
      accLPF[axis] = accLPF[axis] *accDir[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * accDir[axis] * (1.0f/ACC_LPF_FACTOR);
      //accelFloat[axis] = accLPF[axis]; // raw data from int array to float array also give the option to filter if necessary 
    
      static uint32_t previousT;
      uint32_t currentT = microsT1(); // set current time 
      float scale, deltaGyroAngle[3]; 
      //Serial.println();
      scale = 2014 * GYROSCALE; // scalar for raw data GYROSCALE converts raw data to rad/sec then * dt (currentT - previousT)
      
      previousT = currentT; 

      for (axis = 0; axis < 3; axis++){
          deltaGyroAngle[axis] = (gyroADC[axis]) * gyrDir[axis]  * scale * RADTODEG; // (dø/dt - calibration) * dt * constatnt 
        }

      pitchAngle += deltaGyroAngle[0]; //riemann sum
      rollAngle -= deltaGyroAngle[1]; //riemann sum  
      

      float pitchAccel = atan2(accLPF[1],accLPF[2]) * RADTODEG; // determine the pitch of Acc vector using atan2 function
      float rollAccel = atan2(accLPF[0],accLPF[2]) * RADTODEG;  // determine the roll of Acc vector using atan2 function

      accVectorMagnitude = sqrt(pow(accADC[0],2) + pow(accADC[1],2) + pow(accADC[2],2))/495.f; // calc of of Acc vector magnitude, could be simplified 

      if(accVectorMagnitude < 1.3 && accVectorMagnitude > 0.7) // if !bullshit Acc vector must be <1.3g && >0.7g
      {
        if(!(abs(rollAngle) > 85))
          pitchAngle = pitchAngle * GYROWEIGHT + pitchAccel * (1 - GYROWEIGHT); // complementary filter 
        if(!(abs(pitchAngle) > 85))
          rollAngle = rollAngle * GYROWEIGHT + rollAccel * (1 - GYROWEIGHT);    // complementary filter 
      }  

      //serialPrintDebug(pitchAccel); //prints to Serial for debug
    }
