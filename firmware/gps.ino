void generate_setpoint(int way_point_pos){
  double R = 637100000;
  double lat1 = waypoint[way_point_pos].latitude * M_PI/180.0;
  double lat2 = waypoint[way_point_pos + 1].latitude * M_PI/180.0;
  double del_lat = ( waypoint[way_point_pos + 1].latitude - waypoint[way_point_pos].latitude ) * M_PI/180;
  double del_long = ( waypoint[way_point_pos + 1].longitude - waypoint[way_point_pos].longitude ) *M_PI/180;

  double a = (sin(del_lat/2) * sin(del_lat/2)) + (cos(lat1) * cos(lat2) * sin(del_long/2) * sin(del_long/2)) ;
  double azimuth = 2 * atan2(sqrt(a), sqrt(1-a));
  double distance = R * azimuth;

  double theta = atan2((sin(del_long)*cos(lat2)),(cos(lat1) * sin(lat2)) - (sin(lat1)*cos(lat2)*cos(del_long))); 
  previous_theta=theta;
  Serial.print(distance);
  Serial.print(" , ");
  Serial.print(theta*180/M_PI);
  Serial.print(" , ");
  Serial.println(azimuth*180/M_PI);
  delay(10);
  if(!way_point_pos){ 
    arr[way_point_pos][0]=distance*sin(theta);
    arr[way_point_pos][1]=distance*cos(theta);
  }
  else{
    arr[way_point_pos][0]=arr[way_point_pos-1][0]+distance*sin(theta);
    arr[way_point_pos][1]=arr[way_point_pos-1][1]+distance*cos(theta);
  }
}
