void readAngle(){
  
   if (!dmpReady) return;
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();
   fifoCount = mpu.getFIFOCount();

   if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   }

   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }
  
  float degree = ypr[0] * 180/M_PI;
  float radian = ypr[0];
  
  if(degree<0){    degree=360+degree;  }
  if(degree>360){    degree=degree-360;  }
  
  if(radian<0){    radian = 2*PI+radian;  }
  if(radian>2*PI){    radian = radian-2*PI;  }
  
  if(abs(degree-imu.prv_degree)<20){
    imu.degree = degree;
    imu.radian = radian;
  }
  
  imu.prv_degree=degree;
}
