
void encoder_1(){
  l1.t=micros();
  l1.pos=l1_pos;
  if(digitalRead(ENCODER1_B)==HIGH){ l1_pos -=1; }
  else{ l1_pos +=1;  }  
  l1.deltat=(l1.t-l1.prv_t)/1000.0;
  l1.prv_t=l1.t;
  l1.rpm=(60000.0)/(l1.deltat*188.6);
  l1.prv_pos=l1_pos;
}

void encoder_2(){
  r1.t=micros();
  r1.pos=r1_pos;
  if(digitalRead(ENCODER2_B)==HIGH){
    r1_pos -=1;
  }
  else{
    r1_pos +=1;
  }
  r1.deltat=(r1.t-r1.prv_t)/1000.0;
  r1.prv_t=r1.t;
  r1.rpm=(60000.0)/(r1.deltat*188.6);
  r1.prv_pos=r1_pos;
}

void encoder_3(){
  r2.t=micros();
  r2.pos=r2_pos;
  if(digitalRead(ENCODER3_B)==HIGH){
    r2_pos += 1;
  }
  else{
    r2_pos -= 1;
  }
  r2.deltat=(r2.t-r2.prv_t)/1000.0;
  r2.prv_t=r2.t;
  r2.rpm=(60000.0)/(r2.deltat*188.6);
  r2.prv_pos=r2_pos;
}

void encoder_4(){
  
  l2.t=micros();
  l2.pos=l2_pos;
  if(digitalRead(ENCODER4_B)==HIGH){
    l2_pos -=1;
  }
  else{
    l2_pos +=1;
  }
  l2.deltat=(l2.t-l2.prv_t)/1000.0;
  l2.prv_t=l2.t;
  l2.rpm=(60000.0)/(l2.deltat*188.6);
  l2.prv_pos=l2_pos;
}

void stop_motors(){
  analogWrite(L1_PWM,0);
  analogWrite(L2_PWM,0);
  analogWrite(R1_PWM,0);
  analogWrite(R2_PWM,0);
}
