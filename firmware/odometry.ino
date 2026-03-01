void update_bot_location(void){
  global_pos.left_ticks=(l1.pos+l2.pos)/2;
  global_pos.right_ticks=(r1.pos+r2.pos)/2;
  global_pos.orientation= imu.radian;
  global_pos.distance_left=(global_pos.left_ticks-global_pos.prv_left_ticks)*tick_to_cm;
  global_pos.distance_right=(global_pos.right_ticks-global_pos.prv_right_ticks)*tick_to_cm;
  global_pos.distance_center=(global_pos.distance_left+global_pos.distance_right)/2;
  global_pos.x += global_pos.distance_center*sin(global_pos.orientation);
  global_pos.y += global_pos.distance_center*cos(global_pos.orientation);
  global_pos.phi=imu.degree;
  loc.x=global_pos.x;
  loc.y=global_pos.y;
  loc.phi=imu.degree;
  global_pos.prv_left_ticks=global_pos.left_ticks;
  global_pos.prv_right_ticks=global_pos.right_ticks;
  global_pos.prv_orientation=imu.radian; 
}
