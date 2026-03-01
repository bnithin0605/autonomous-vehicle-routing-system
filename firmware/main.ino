//THE FOLLOWING CODE HAS MODIFIED REPULSIVE FEILDS IN ADDITION TO ALL THE FEATURES OF VERSION 6

#include <TimerOne.h>
#include <TimerThree.h>

const float wheel_seperation=43;
const float wheel_diamter=12.5;
const float tick_to_cm=0.20811240721;
const float rad_to_degree=57.2957795131;
const float rpm_to_rad_per_sec=0.10472;
const float rad_per_sec_to_rpm=9.549297;
const float rpm_to_cm_per_sec=1.5286;
const float kp=0.8;
const float ki=0.025;

// HIGH speeds kp=1.2, ki =0.025
// LOW speeds kp=0.4, ki=0.025

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define ENCODER1_A 3
#define ENCODER1_B 34
#define ENCODER2_A 18
#define ENCODER2_B 36
#define ENCODER3_A 2
#define ENCODER3_B 32
#define ENCODER4_A 19
#define ENCODER4_B 38

#define L1_DIR 33
#define L2_DIR 37
#define R1_DIR 35
#define R2_DIR 39

#define L1_PWM 4
#define L2_PWM 6
#define R1_PWM 5
#define R2_PWM 7

#define MIN_PWM 0

#define OUTPUT_READABLE_YAWPITCHROLL

#define q_inf 70
#define q_min 30
#define repulsion_scaling 500

#define minimum_angular_velocity 0.1

// STRUCTURES AND VARIABLES

MPU6050 mpu;
bool blinkState = false;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;   
uint8_t fifoBuffer[64]; 

Quaternion q;    
VectorInt16 aa; 
VectorInt16 gy;    
VectorInt16 aaReal;   
VectorInt16 aaWorld;  
VectorFloat gravity; 
float euler[3];    
float ypr[3]; 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

bool state, prv_state;
volatile bool mpuInterrupt = false;    

volatile int l1_pos,l2_pos,r1_pos,r2_pos;
unsigned long int t, rpm_set_time, prv_rpm_set_time, set_point_time, minima_start_time;
float linear_velocity, angular_velocity;
float target_angle;
float t_x,t_y;
int arr_pos,hex_arr_pos;
float u,v, minima_theta, rightside_distance, leftside_distance;
int minima_flag, minima_x, minima_y;
int arr[4][3];
int hex_arr[3][3];
double previous_theta;
struct ultraSonic{
  float distance;
  bool state, prv_state;
  unsigned long int first,last;
}ultrasonic[8];

struct mpu_angle{
  float degree;
  float radian;
  float prv_degree;
  float prv_radian;
}imu;

struct motor{
  unsigned long int t,prv_t=0;
  float pos,prv_pos=0;
  float deltat,rpm,prv_rpm,rpm_error,prv_rpm_error=0;
  int power,prv_power=0;
  float rpm_setpoint=0;
  float min_error,max_error;
}l1,l2,r1,r2;

struct wheel_velocities{
  float left_wheel_velocities, right_wheel_velocities;
}ik;

struct global_position{
  float left_ticks,prv_left_ticks,right_ticks,prv_right_ticks;
  float distance_left, distance_center, distance_right;
  float orientation, prv_orientation;
  float x, y, phi;
}global_pos;

struct bot_location{
  int x;
  int y;
  float phi;
}loc;

struct ostacle_cordinates{
  int x;
  int y;
}obs[5];

struct gps_waypoint{
  float latitude;
  float longitude;
}waypoint[5];

//FUNCTIONS AND FUNCTION PROTOTYPES
void mpu_int(){
  state=digitalRead(12);
  if(state==1 && prv_state==0){
    dmpDataReady();
  }
  prv_state=state;
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void readAngle();

//VOID SETUP
void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
mpu.initialize();

pinMode(12,INPUT);
pinMode(ENCODER1_A,INPUT);
pinMode(ENCODER1_B,INPUT);

pinMode(ENCODER2_A,INPUT);
pinMode(ENCODER2_B,INPUT);

pinMode(ENCODER3_A,INPUT);
pinMode(ENCODER3_B,INPUT);

pinMode(ENCODER4_A,INPUT);
pinMode(ENCODER4_B,INPUT);

Timer1.initialize(10000);
Timer1.attachInterrupt(mpu_int);

Timer3.initialize(50);
Timer3.attachInterrupt(ultrasonic_sensor);

attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder_1, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder_2, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder_3, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER4_A), encoder_4, RISING);

pinMode(LED_BUILTIN, OUTPUT);
Serial.begin(9600);

while(!mpu.testConnection()){Serial.println(mpu.testConnection());}
Serial.print("connection succesful");
devStatus = mpu.dmpInitialize();
mpu.setXGyroOffset(51);
mpu.setYGyroOffset(8);
mpu.setZGyroOffset(21);
mpu.setXAccelOffset(1150);
mpu.setYAccelOffset(-50);
mpu.setZAccelOffset(1060);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(10, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
else{
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
}

digitalWrite(L1_DIR,LOW);
digitalWrite(L2_DIR,LOW);
digitalWrite(R1_DIR,LOW);
digitalWrite(R2_DIR,LOW);


waypoint[0].latitude = 16.4433449;
waypoint[0].longitude = 80.6206316;

waypoint[1].latitude = 16.4435404;
waypoint[1].longitude = 80.6206316;

waypoint[2].latitude = 16.4435469;
waypoint[2].longitude = 80.6204492;

int i=0;
while(i<=1){
  generate_setpoint(i);
  i=i+1;
}
arr_pos=0;
t_x=arr[arr_pos][0];
t_y=arr[arr_pos][1];

linear_velocity=0;
angular_velocity=0;

pinMode(45,OUTPUT);
pinMode(46,INPUT);
pinMode(47,INPUT);
pinMode(48,INPUT);
pinMode(49,INPUT);
pinMode(50,INPUT); 
 
Serial.print("( ");
Serial.print(arr[0][0]);
Serial.print(",");
Serial.print(arr[0][1]);
Serial.print(" ) , "); 
Serial.print("( ");
Serial.print(arr[1][0]);
Serial.print(",");
Serial.print(arr[1][1]);
Serial.print(" ) , ");
Serial.print("( ");
Serial.print(arr[2][0]);
Serial.print(",");
Serial.print(arr[2][1]);
Serial.print(" ) , ");
Serial.print("( ");
Serial.print(arr[3][0]);
Serial.print(",");
Serial.print(arr[3][1]);
Serial.println(" ) .");
delay(5000);
rpm_set_time=millis();
t=millis();
set_point_time=millis();
}


void loop() {
  if(micros()-l1.t>100000){ l1.rpm=0; }
  if(micros()-l2.t>100000){ l2.rpm=0; }
  if(micros()-r1.t>100000){ r1.rpm=0; }
  if(micros()-r2.t>100000){ r2.rpm=0; }

  digitalWrite(45,LOW);
  delayMicroseconds(2);
  digitalWrite(45,HIGH);
  delayMicroseconds(5);
  digitalWrite(45,LOW);

  generate_obstacle_cordinates();

  readAngle();

  update_bot_location();

//SETPOINT REGION
if(minima_flag<=1){
  t_x=arr[arr_pos][0];
  t_y=arr[arr_pos][1];
}
else{
  t_x=hex_arr[hex_arr_pos][0];
  t_y=hex_arr[hex_arr_pos][1];
}

float target_distance = (((t_x-loc.x)*(t_x-loc.x))+((t_y-loc.y)*(t_y-loc.y)));
float distance_uncapped=target_distance;
u=2*(t_x-loc.x);
v=2*(t_y-loc.y);
if(target_distance>47){target_distance=47;}

float net_force=target_distance;
float min_distance=500;
int k=0;
while(k<=4){
  if((ultrasonic[k].distance < q_inf) && (ultrasonic[k].distance!=0)){
    u = u-250*(obs[k].x)/ultrasonic[k].distance;
    v = v-250*(obs[k].y)/ultrasonic[k].distance;
    
    u = u-250*(obs[k].x)/ultrasonic[k].distance;
    v = v-250*(obs[k].y)/ultrasonic[k].distance;
    
    u = u-250*(obs[k].x)/ultrasonic[k].distance;
    v = v-250*(obs[k].y)/ultrasonic[k].distance;
    
    u = u-250*(obs[k].x)/ultrasonic[k].distance;
    v = v-250*(obs[k].y)/ultrasonic[k].distance;    
    
    if(ultrasonic[k].distance < min_distance){
      min_distance = ultrasonic[k].distance;
    }
  
  }
  k=k+1;
}
target_angle = (90-(atan2(v,u)*rad_to_degree));

if(target_angle<0){
  target_angle=360+target_angle;
}
if(target_angle>360){
  target_angle=target_angle-360;
}

if((ultrasonic[0].distance<=20) && (ultrasonic[1].distance<=20) && (ultrasonic[2].distance<=20) && (minima_flag==0)){ 
  minima_flag=1;
  minima_x=int(loc.x);
  minima_y=int(loc.y);
  minima_theta=imu.radian; 
  minima_start_time=millis(); 
}

//ASSIGNING ANGULAR_VELOCITY
float angle_error= target_angle-loc.phi;
float clockwise_angle_error, anti_clockwise_angle_error=0;

if(abs(angle_error)>0.35){
    if(target_angle<loc.phi){
      clockwise_angle_error=360+target_angle-loc.phi;
    }
    else if(target_angle>=loc.phi){
        clockwise_angle_error=target_angle-loc.phi;
    }
    anti_clockwise_angle_error=360-clockwise_angle_error;
    
    if(clockwise_angle_error<anti_clockwise_angle_error){
      clockwise_angle_error=clockwise_angle_error/rad_to_degree;
      if(clockwise_angle_error<0.3){clockwise_angle_error=0.3;}
      if(clockwise_angle_error>3.0){clockwise_angle_error=3.0;}
      angular_velocity=clockwise_angle_error;
    }
    else if(clockwise_angle_error>anti_clockwise_angle_error){
      anti_clockwise_angle_error=anti_clockwise_angle_error/rad_to_degree;
      if(anti_clockwise_angle_error<0.3){anti_clockwise_angle_error=0.3;}
      if(anti_clockwise_angle_error>3.0){anti_clockwise_angle_error=3.0;}
      angular_velocity=-anti_clockwise_angle_error;
    }  
}
else{
  angular_velocity=0;
}

//ASSIGNING LINEAR_VELOCITY AND REMOVING LOCAL MINIMA
if(!minima_flag){
if(target_distance<=25 && arr_pos==1){
  linear_velocity=0;
  angular_velocity=0;
  l1.prv_power=0;  l2.prv_power=0;  r1.prv_power=0;  r2.prv_power=0;
  l1.power=0;  l2.power=0;  r1.power=0;  r2.power=0;
  stop_motors();
}

if(target_distance<=25 && arr_pos<1){
  arr_pos=arr_pos+1;
  linear_velocity=target_distance;
}
else{
  if(min_distance>target_distance){
    linear_velocity=target_distance;
  }
  else{
    linear_velocity=min_distance;
  }
}

}

else if(minima_flag==1){
    if((millis()- minima_start_time)<=10000){
    linear_velocity=-10;
    angular_velocity=0;    
    rightside_distance=ultrasonic[1].distance + ultrasonic[3].distance;
    leftside_distance=ultrasonic[2].distance + ultrasonic[4].distance;
  }
  else{
    minima_flag=2;
    if(rightside_distance<=leftside_distance){
      float l_ang  = minima_theta - M_PI/3;
      if(l_ang>2*M_PI){l_ang=l_ang-2*M_PI;}
      if(l_ang<0){l_ang=2*M_PI+l_ang;}
      hex_arr[0][0] = minima_x + 100*sin(l_ang);
      hex_arr[0][1] = minima_y + 100*cos(l_ang);
      
      hex_arr[1][0] = hex_arr[0][0] + 100*sin(minima_theta);
      hex_arr[1][1] = hex_arr[0][1] + 100*cos(minima_theta);

      l_ang  = minima_theta + M_PI/3;
      if(l_ang>2*M_PI){l_ang=l_ang-2*M_PI;}
      if(l_ang<0){l_ang=2*M_PI+l_ang;}
      hex_arr[2][0] = hex_arr[1][0] + 100*sin(l_ang);
      hex_arr[2][1] = hex_arr[1][1] + 100*cos(l_ang);


    }
    else{
      float r_ang  = minima_theta + M_PI/3;
      if(r_ang>2*M_PI){r_ang=r_ang-2*M_PI;}
      if(r_ang<0){r_ang=2*M_PI+r_ang;}
      hex_arr[0][0] = minima_x + 100*sin(r_ang);
      hex_arr[0][1] = minima_y + 100*cos(r_ang);
      
      hex_arr[1][0] = hex_arr[0][0] + 100*sin(minima_theta);
      hex_arr[1][1] = hex_arr[0][1] + 100*cos(minima_theta);

      r_ang  = minima_theta - M_PI/3;
      if(r_ang>2*M_PI){r_ang=r_ang-2*M_PI;}
      if(r_ang<0){r_ang=2*M_PI+r_ang;}
      hex_arr[2][0] = hex_arr[1][0] + 100*sin(r_ang);
      hex_arr[2][1] = hex_arr[1][1] + 100*cos(r_ang);
    }
  }
}

else if(minima_flag==2){
  if(target_distance<=25 && hex_arr_pos<2){
    hex_arr_pos+=1;
  }
  else if(target_distance<=25 && hex_arr_pos==2) {
    hex_arr_pos=0;
    minima_flag=0;
  }
  if(target_distance>=23){
    linear_velocity=23;
  }
  else{
  linear_velocity=target_distance;
  }
}
Serial.print(target_angle);
Serial.print(", ");
Serial.print(angular_velocity);
Serial.print(", ");
Serial.print(linear_velocity);
Serial.println(" . ");
//DIFFEENTIAL DRIVE IK REGION
if((millis()-t)>10){
  generate_motor_speeds();
  }
  
//PID GOVERNOR REGION
 if((millis()-rpm_set_time)>100){
  pid_rpm_control();
 }  

delay(5);
}
