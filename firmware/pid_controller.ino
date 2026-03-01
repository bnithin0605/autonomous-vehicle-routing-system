void pid_rpm_control(void){
  rpm_set_time=millis();
  l1.rpm_error=l1.rpm_setpoint-l1.rpm;
  l2.rpm_error=l2.rpm_setpoint-l2.rpm;
  r1.rpm_error=r1.rpm_setpoint-r1.rpm;
  r2.rpm_error=r2.rpm_setpoint-r2.rpm;
  
  l1.power=int (kp * l1.rpm_error) + l1.prv_power + (ki*(l1.rpm_error + l1.prv_rpm_error));
  l2.power=int (kp * l2.rpm_error) + l2.prv_power + (ki*(l2.rpm_error + l2.prv_rpm_error));
  r1.power=int (kp * r1.rpm_error) + r1.prv_power + (ki*(r1.rpm_error + r1.prv_rpm_error));
  r2.power=int (kp * r2.rpm_error) + r2.prv_power + (ki*(r2.rpm_error + r2.prv_rpm_error));

  l1.prv_rpm_error=l1.rpm_error;
  l2.prv_rpm_error=l2.rpm_error;
  r1.prv_rpm_error=r1.rpm_error;
  r2.prv_rpm_error=r2.rpm_error;
  
  l1.prv_power=l1.power;
  if(l1.power>225){l1.power=225;}
  if(l1.power<MIN_PWM){l1.power=MIN_PWM;}

  l2.prv_power=l2.power;
  if(l2.power>225){l2.power=225;}
  if(l2.power<MIN_PWM){l2.power=MIN_PWM;}
  
  r1.prv_power=r1.power;
  if(r1.power>225){r1.power=225;}
  if(r1.power<MIN_PWM){r1.power=MIN_PWM;}
  
  r2.prv_power=r2.power;
  if(r2.power>225){r2.power=225;}
  if(r2.power<MIN_PWM){r2.power=MIN_PWM;}
  
  analogWrite(L1_PWM,int(l1.power));
  analogWrite(L2_PWM,int(l2.power));
  analogWrite(R1_PWM,int(r1.power));
  analogWrite(R2_PWM,int(r2.power));
}
