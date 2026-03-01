void generate_motor_speeds(void){
    t=millis();
    ik.left_wheel_velocities=( (2*linear_velocity)+(angular_velocity*wheel_seperation) )/(wheel_diamter);
    ik.right_wheel_velocities=( (2*linear_velocity)-(angular_velocity*wheel_seperation) )/(wheel_diamter);

    if(ik.left_wheel_velocities>=0){
      digitalWrite(L1_DIR,HIGH);
      digitalWrite(L2_DIR,HIGH);
      l1.rpm_setpoint=ik.left_wheel_velocities*rad_per_sec_to_rpm;
      l2.rpm_setpoint=ik.left_wheel_velocities*rad_per_sec_to_rpm;
    }
    else{
      digitalWrite(L1_DIR,LOW);
      digitalWrite(L2_DIR,LOW);
      l1.rpm_setpoint=-ik.left_wheel_velocities*rad_per_sec_to_rpm;
      l2.rpm_setpoint=-ik.left_wheel_velocities*rad_per_sec_to_rpm;
    }
    if(ik.right_wheel_velocities>=0){
      digitalWrite(R1_DIR,HIGH);
      digitalWrite(R2_DIR,HIGH);
      r1.rpm_setpoint=ik.right_wheel_velocities*rad_per_sec_to_rpm;
      r2.rpm_setpoint=ik.right_wheel_velocities*rad_per_sec_to_rpm;
    }
    else{
      digitalWrite(R1_DIR,LOW);
      digitalWrite(R2_DIR,LOW);
      r1.rpm_setpoint=-ik.right_wheel_velocities*rad_per_sec_to_rpm;
      r2.rpm_setpoint=-ik.right_wheel_velocities*rad_per_sec_to_rpm;
    }
}
