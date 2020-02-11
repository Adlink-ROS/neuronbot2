#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
//#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "math.h"
#include "ros/console.h"
#include "dynamic_reconfigure/server.h"
#include "filter/MyStuffConfig.h"


using namespace std_msgs;

geometry_msgs::QuaternionStamped q;
geometry_msgs::Vector3Stamped v;
geometry_msgs::TransformStamped q_trans;
double twoKp;  //2* accelerometer proportional gain
double twoKi;  //2* integral gain
float sampleFreq;
float q0=1.0, q1=0.0, q2=0.0, q3=0.0;
float ex_int=0.0, ey_int=0.0, ez_int=0.0;
Header header;
float ax, ay, az, gx, gy, gz;
ros::Duration dtime;
float dt;


float invSqrt(float );
void qua2Euler(geometry_msgs::QuaternionStamped );
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void callback(filter::MyStuffConfig &config, uint32_t level) {
  twoKp = config.twoKp;
  twoKi = config.twoKi;
  ROS_INFO("twoKi=%f, twoKp=%f", twoKi, twoKp);
}


//Implementation of Mahony&Madgwick's IMU and AHRS algorithms.
//reference: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
void filter_function(const sensor_msgs::Imu& msg)
{
 // ros::Time start_time = ros::Time::now();

  header = msg.header;
//  q.linear_acceleration = msg.linear_acceleration;
//  q.angular_velocity = msg.angular_velocity;
  ax = msg.linear_acceleration.x;
  ay = msg.linear_acceleration.y;
  az = msg.linear_acceleration.z;
  gx = msg.angular_velocity.x;
  gy = msg.angular_velocity.y;
  gz = msg.angular_velocity.z;

/*  ros::Time this_time = header.stamp;
  static ros::Time last_time = header.stamp;
  dtime = this_time - last_time;
  dt = dtime.toNSec();
  ROS_INFO(" this_time = %f", this_time.toSec());
  ROS_INFO(" last_time = %f", last_time.toSec());
  dt = dt/1.0e9;
  // dt = 0.0025;  
  ROS_INFO(" dt = %f", dt);
  last_time = this_time;   */
 
  // since we are using 1000 degrees/seconds range and the register is 16 bits
  // -1000 maps to a raw value of -32768
  // +1000 maps to a raw value of 32767
  gx = gx*1000.0/32768.0;
  gy = gy*1000.0/32768.0;
  gz = gz*1000.0/32768.0;

  //change Gyroscope units to radians/second, accelerometer units are irrelevant as the vector is normalised.
  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  ROS_INFO("The original gx=%f, gy=%f, gz=%f ", gx, gy, gz);
  
  MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
 
  q.header = header;
  q.quaternion.w = q0;
  q.quaternion.x = q1;
  q.quaternion.y = q2;
  q.quaternion.z = q3;

  ROS_INFO(" q0=%f, q1=%f, q2=%f, q3=%f ", q0, q1, q2, q3);
  qua2Euler(q); 
 //  ros::Time end_time = ros::Time::now();
//  double interval_time = (end_time - start_time).toNSec();
 // ROS_INFO("the  interval time is %lf ns" , interval_time);
}

  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mahony_filter");
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<geometry_msgs::QuaternionStamped>("quaternion", 1);
  ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3Stamped>("ypr", 1);
 // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster q_broadcaster;
  //ros::Publisher pub3 = n.advertise<std_msgs::Float64MultiArray>("DCM",20);
  //setFullSacleGyroRange(ICM20602_GYRO_RANGE_1000);
  ros::Subscriber sub = n.subscribe("/imu/data_raw", 10, filter_function);
  if(!n.getParam("sampleFreq", sampleFreq))
    sampleFreq = 400.0;
  dt = 1.0/sampleFreq;

  dynamic_reconfigure::Server<filter::MyStuffConfig> server;
  dynamic_reconfigure::Server<filter::MyStuffConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Rate r(sampleFreq);
  while(ros::ok())
  {
    pub1.publish(q);
    pub2.publish(v);
    ros::spinOnce();
   
    q_trans.header.stamp = header.stamp;
    q_trans.header.frame_id = "/odom";
    q_trans.child_frame_id = "/imu";
/*  q_trans.transform.rotation.w = q.quaternion.w;
  q_trans.transform.rotation.x = q.quaternion.x;
  q_trans.transform.rotation.y = q.quaternion.y;
  q_trans.transform.rotation.z = q.quaternion.z; */
  q_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(v.vector.z ,v.vector.y, v.vector.x);

  q_broadcaster.sendTransform(q_trans);
  r.sleep();
  }
  return 0;
}

//Fast inverse square root
//reference: https://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float xhalf = 0.5f * x;
  union
  {
    float x;
    int i;
  } u;
  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  /* The next line can be repeated any number of times to increase accuracy */
  u.x = u.x * (1.5f - xhalf * u.x * u.x);
  return u.x;
}

//reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void qua2Euler(geometry_msgs::QuaternionStamped q) {

  float x,y,z,w;
  
  x = q.quaternion.x;
  y = q.quaternion.y;
  z = q.quaternion.z;
  w = q.quaternion.w;

  v.header = q.header;

//yaw (z-axis rotation)
  float t0 = 2.0*(w*z+x*y);
  float t1 = 1.0-2.0*(y*y+z*z);
  v.vector.x = atan2(t0,t1)*57.29578; //the unit is degree
//pitch (y-axis rotation)
  float t2 = 2.0*(w*y-z*x);
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0)? -1.0 : t2;
  v.vector.y = asin(t2)*57.29578;  
//roll (x-axis rotation)
  float t3 = 2.0*(w*x+y*z);
  float t4 = 1.0-2.0*(x*x+y*y);
  v.vector.z = atan2(t3, t4)*57.29578;

} 

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {

  float recipNorm; 
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
   float thetax, thetay, thetaz;
  float dq0, dq1, dq2, dq3;

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
  //normalize the accelerometer vector
  recipNorm = invSqrt(ax*ax+ay*ay+az*az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;  

  ROS_INFO(" ax=%f, ay=%f, az=%f ", ax, ay, az);

  //calculate the direction of gravity according to quaternion
  halfvx = q1*q3 - q0*q2;
  halfvy = q0*q1 + q2*q3;
// vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  halfvz = q0*q0 -0.5f + q3*q3;

  ROS_INFO("The halfvx=%f, halfvy=%f, halfvz=%f", halfvx, halfvy, halfvz);

  

  //calculate the error which is cross product between estimated and measured direction of gravity
  halfex = ay*halfvz - az*halfvy;
  halfey = az*halfvx - ax*halfvz;
  halfez = ax*halfvy - ay*halfvx; 

  ROS_INFO("The halfex=%f, halfey=%f, halfez=%f", halfex, halfey, halfez);

  if (twoKi > 0.0f) {
  //integral the error
  ex_int += twoKi*halfex*dt;
  ey_int += twoKi*halfey*dt;
  ez_int += twoKi*halfez*dt;

  ROS_INFO("The ex_int=%f, ey_int=%f, ez_int=%f", ex_int, ey_int, ez_int);

  //apply the integral feedback
  gx += ex_int;
  gy += ey_int;
  gz += ez_int;  }

  else {
  ex_int = 0.0f;
  ey_int = 0.0f;
  ez_int = 0.0f;
  }

  //apply the proportional feedback
  gx += twoKp*halfex;
  gy += twoKp*halfey;
  gz += twoKp*halfez;  
  
  ROS_INFO("The revised gx=%f, gy=%f, gz=%f ", gx, gy, gz);  }

  //calculate the rotary angle
  thetax = 0.5*gx*dt;
  thetay = 0.5*gy*dt;
  thetaz = 0.5*gz*dt;

  //update quaternion using Runge Kutta algorithms.
  //explanation: http://blog.csdn.net/aasdsadad/article/details/73080832
  dq0 = -q1*thetax-q2*thetay-q3*thetaz;
  dq1 = q0*thetax+q2*thetaz-q3*thetay;
  dq2 = q0*thetay-q1*thetaz+q3*thetax;
  dq3 = q0*thetaz+q1*thetay-q2*thetax;

  q0 += dq0;
  q1 += dq1;
  q2 += dq2;
  q3 += dq3;

  //normalize quaternion
  recipNorm = invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm; 

}


