void generate_obstacle_cordinates(){
  obs[0].x =   0 + /*loc.x +*/ (ultrasonic[0].distance)*sin(imu.radian);
  obs[0].y =  20 + /*loc.y +*/ (ultrasonic[0].distance)*cos(imu.radian);
  
  obs[1].x =  12 + /*loc.x +*/ (ultrasonic[1].distance)*sin(0.35 + imu.radian);
  obs[1].y =  20 + /*loc.y +*/ (ultrasonic[1].distance)*cos(0.35 + imu.radian);
  
  obs[2].x = -12 + /*loc.x +*/ (ultrasonic[2].distance)*sin((-0.35) + imu.radian);
  obs[2].y =  20 + /*loc.y +*/ (ultrasonic[2].distance)*cos((-0.35) + imu.radian);
  
  obs[3].x =  12 + /*loc.x +*/ (ultrasonic[3].distance)*sin(M_PI/2 + imu.radian);
  obs[3].y =   0 + /*loc.y +*/ (ultrasonic[3].distance)*cos(M_PI/2 + imu.radian);
  
  obs[4].x = -12 + /*loc.x +*/ (ultrasonic[4].distance)*sin(-M_PI/2 + imu.radian);
  obs[4].y =   0 + /*loc.y +*/ (ultrasonic[4].distance)*cos(-M_PI/2 + imu.radian);
}

void ultrasonic_sensor(){
  ultrasonic[0].state=digitalRead(46);
  ultrasonic[1].state=digitalRead(47);
  ultrasonic[2].state=digitalRead(48);
  ultrasonic[3].state=digitalRead(49);
  ultrasonic[4].state=digitalRead(50);
  
  if(!ultrasonic[0].prv_state && ultrasonic[0].state){    ultrasonic[0].first=micros();}
  if(!ultrasonic[1].prv_state && ultrasonic[1].state){    ultrasonic[1].first=micros();}
  if(!ultrasonic[2].prv_state && ultrasonic[2].state){    ultrasonic[2].first=micros();}
  if(!ultrasonic[3].prv_state && ultrasonic[3].state){    ultrasonic[3].first=micros();}
  if(!ultrasonic[4].prv_state && ultrasonic[4].state){    ultrasonic[4].first=micros();}
  
  if(ultrasonic[0].prv_state && !ultrasonic[0].state){
    ultrasonic[0].last=micros();
    ultrasonic[0].distance=(ultrasonic[0].last-ultrasonic[0].first)/29/2;
  }
  if(ultrasonic[1].prv_state && !ultrasonic[1].state){
    ultrasonic[1].last=micros();
    ultrasonic[1].distance=(ultrasonic[1].last-ultrasonic[1].first)/29/2;
  }
  if(ultrasonic[2].prv_state && !ultrasonic[2].state){
    ultrasonic[2].last=micros();
    ultrasonic[2].distance=(ultrasonic[2].last-ultrasonic[2].first)/29/2;
  }
  if(ultrasonic[3].prv_state && !ultrasonic[3].state){
    ultrasonic[3].last=micros();
    ultrasonic[3].distance=(ultrasonic[3].last-ultrasonic[3].first)/29/2;
  }
  if(ultrasonic[4].prv_state && !ultrasonic[4].state){
    ultrasonic[4].last=micros();
    ultrasonic[4].distance=(ultrasonic[4].last-ultrasonic[4].first)/29/2;
  }  
  ultrasonic[0].prv_state=ultrasonic[0].state;
  ultrasonic[1].prv_state=ultrasonic[1].state;
  ultrasonic[2].prv_state=ultrasonic[2].state;
  ultrasonic[3].prv_state=ultrasonic[3].state;
  ultrasonic[4].prv_state=ultrasonic[4].state;
}
