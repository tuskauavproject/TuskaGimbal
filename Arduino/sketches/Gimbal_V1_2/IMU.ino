void IMU(){
  for(axis = 0; axis < 3; axis++)
      accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
      //accelFloat[axis] = accLPF[axis]; // raw data from int array to float array also give the option to filter if necessary 
    
      static uint16_t previousT;
      uint16_t currentT = micros(); // set current time 
      float scale, deltaGyroAngle[3]; 
       
      scale = (currentT - previousT) * GYROSCALE; // scalar for raw data GYROSCALE converts raw data to rad/sec then * dt
      previousT = currentT; 

      for (axis = 0; axis < 3; axis++){
          deltaGyroAngle[axis] = (gyroADC[axis])  * scale * RADTODEG; // (dø/dt - calibration) * dt * constatnt 
        }
    
      angleSum[0] += deltaGyroAngle[0]; // for debug remove!!!!!!!!!!!!! 

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


      if(pitchAngle > 180)
        pitchAngle = -180 + (pitchAngle - 180);
      else if(pitchAngle < -180)
        pitchAngle = 180 + (pitchAngle + 180);

      if(rollAngle > 180)
        rollAngle = -180 + (rollAngle - 180);
      else if(rollAngle < -180)
        rollAngle = 180 + (rollAngle + 180);

      //serialPrintDebug(pitchAccel); //prints to Serial for debug
    }