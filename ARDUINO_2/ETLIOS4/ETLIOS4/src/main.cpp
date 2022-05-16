#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <MeMegaPi.h>
#define DEFAULT_SAMPLE_RATE_HZ	10

#define MPU_FRAMEID "base_imu"


ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::Imu imu_msg;
geometry_msgs::Point rpms_real_msg;
ros::Publisher imu_data("imu",&imu_msg);
ros::Publisher rpm_real("rpm_real",&rpms_real_msg);

MeEncoderOnBoard leftEnc(SLOT4);
MeEncoderOnBoard rightEnc(SLOT2);

void isr_process_left(void){
  if(digitalRead(leftEnc.getPortB()) == 0){
    leftEnc.pulsePosMinus();
  }else{
    leftEnc.pulsePosPlus();
  }
}
void isr_process_right(void){
  if(digitalRead(rightEnc.getPortB()) == 0){
    rightEnc.pulsePosMinus();
  }else{
    rightEnc.pulsePosPlus();
  }
}


void rpms_wanted_callback(const geometry_msgs::Point& rpm_wanted_msg)
{
    leftEnc.runSpeed(rpm_wanted_msg.x);
    rightEnc.runSpeed(rpm_wanted_msg.y); 
}


ros::Subscriber<geometry_msgs::Point> rpm_wanted("rpm_wanted",&rpms_wanted_callback);


double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;
bool debug = false;

void rosSetup() {
  // put your setup code here, to run once:
   nh.initNode();
   nh.advertise(imu_data);
   nh.advertise(rpm_real);
   nh.subscribe(rpm_wanted);

 
}

void motorSetup()
{
  attachInterrupt(leftEnc.getIntNum(), isr_process_left, RISING);
  leftEnc.setPulse(8);
  leftEnc.setRatio(46.67);
  leftEnc.setPosPid(1.8,0,1.2);
  leftEnc.setSpeedPid(0.18,0,0);
  attachInterrupt(rightEnc.getIntNum(), isr_process_right, RISING);
  rightEnc.setPulse(8);
  rightEnc.setRatio(46.67);
  rightEnc.setPosPid(1.8,0,1.2);
  rightEnc.setSpeedPid(0.18,0,0);
}


#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  rosSetup();
  
  Wire.begin();
  
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
        // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
  linear_acceleration_stdev_ = (400 / 1000000.0) * 9.807 ;
  angular_velocity_stdev_ =  0.05 * (M_PI / 180.0);
  pitch_roll_stdev_ = 1.0 * (M_PI / 180.0);
  yaw_stdev_ = 5.0 * (M_PI / 180.0);



  angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
  linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
  pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
  yaw_covariance = yaw_stdev_ * yaw_stdev_;
  motorSetup();
  
}

void imuLoop()
{
  mpu.update();
    ros::Time now = nh.now();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_frame";
    geometry_msgs::Quaternion q = tf::createQuaternionFromYaw(mpu.getAngleZ() * DEG_TO_RAD);
    imu_msg.orientation.x = q.x;
		imu_msg.orientation.y = q.y;
		imu_msg.orientation.z = q.z;
		imu_msg.orientation.w = q.w;
      
    imu_msg.angular_velocity.x = mpu.getGyroX()*DEG_TO_RAD;
    imu_msg.angular_velocity.y = mpu.getGyroY()*DEG_TO_RAD;
    imu_msg.angular_velocity.z = mpu.getGyroZ()*DEG_TO_RAD;
		
    imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
		imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
		imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

		imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance;
		imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance;
		imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance;

		imu_msg.orientation_covariance[0] = pitch_roll_covariance;
		imu_msg.orientation_covariance[4] = pitch_roll_covariance;
		imu_msg.orientation_covariance[8] = yaw_covariance;

    imu_msg.linear_acceleration.x = mpu.getAccX() * 9.80665;
    imu_msg.linear_acceleration.y = mpu.getAccY()* 9.80665;
    imu_msg.linear_acceleration.z =0; mpu.getAccZ()* 9.80665;
     imu_data.publish(&imu_msg);
    
  
}

void motorLoop()
{
  leftEnc.loop();
  rightEnc.loop();
  rpms_real_msg.x = -leftEnc.getCurrentSpeed();
  rpms_real_msg.y = rightEnc.getCurrentSpeed();
  rpm_real.publish(&rpms_real_msg);
}

void loop() {
  
  imuLoop();
  motorLoop();
  nh.spinOnce();
  delay(20);


}