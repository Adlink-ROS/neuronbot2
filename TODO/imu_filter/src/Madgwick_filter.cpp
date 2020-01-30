#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
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
#include "filter/MyStuff2Config.h"


using namespace std_msgs;

geometry_msgs::QuaternionStamped q;
geometry_msgs::Vector3Stamped v;
geometry_msgs::TransformStamped q_trans;
float sampleFreq;
double beta; 
float q0=1.0, q1=0.0, q2=0.0, q3=0.0;
Header header;
float ax, ay, az, gx, gy, gz;
ros::Duration dtime;
float dt;

float invSqrt(float );
void qua2Euler(geometry_msgs::QuaternionStamped );
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void callback(filter::MyStuff2Config &config, uint32_t level) {
  beta = config.beta;

}


void filter_function(const sensor_msgs::Imu& msg)
{
//  ros::Time start_time = ros::Time::now();


  header = msg.header;
//  q.linear_acceleration = msg.linear_acceleration;
//  q.angular_velocity = msg.angular_velocity;
  ax = msg.linear_acceleration.x;
  ay = msg.linear_acceleration.y;
  az = msg.linear_acceleration.z;
  gx = msg.angular_velocity.x;
  gy = msg.angular_velocity.y;
  gz = msg.angular_velocity.z;

 /* ros::Time this_time = header.stamp;
  static ros::Time last_time = header.stamp;
  dtime = this_time - last_time;
  dt = dtime.toNSec();
  ROS_INFO(" this_time = %f", this_time.toSec());
  ROS_INFO(" last_time = %f", last_time.toSec());
  dt = dt/1.0e9;
  // dt = 0.0025;  
  ROS_INFO(" dt = %f", dt);
  last_time = this_time; */
 
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
  ROS_INFO("beta=%f", beta);

  MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az) ;

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
  tf::TransformBroadcaster q_broadcaster;
  ros::Subscriber sub = n.subscribe("/imu/data_raw", 10, filter_function);
  if(!n.getParam("sampleFreq", sampleFreq))
    sampleFreq = 400.0;
  dynamic_reconfigure::Server<filter::MyStuff2Config> server;
  dynamic_reconfigure::Server<filter::MyStuff2Config>::CallbackType f;
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
  v.vector.x = atan2(t0,t1); //the unit is radians
//pitch (y-axis rotation)
  float t2 = 2.0*(w*y-z*x);
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0)? -1.0 : t2;
  v.vector.y = asin(t2);  
//roll (x-axis rotation)
  float t3 = 2.0*(w*x+y*z);
  float t4 = 1.0-2.0*(x*x+y*y);
  v.vector.z = atan2(t3, t4);
} 


//Implementation of Madgwick's IMU and AHRS algorithms.
//reference: An efficient orientation filter for inertial and inertial/magnetic sensor arrays. (This thesis's appendix has more detailed code and illutration)
//reference: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {  
  float recipNorm;    
  float s0, s1, s2, s3;   // estimated direction of the gyroscope error
  float qDot1, qDot2, qDot3, qDot4;    // quaternion derrivative from gyroscopes elements
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;  //Axulirary variables to avoid reapeated calcualtions
      
  // Rate of change of quaternion from gyroscope  
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);  
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);  
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);  
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);  
      
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)  
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {  
      
     // Normalise accelerometer measurement  
     recipNorm = invSqrt(ax * ax + ay * ay + az * az);  
     ax *= recipNorm;  
     ay *= recipNorm;  
     az *= recipNorm;     
      
     // Auxiliary variables to avoid repeated arithmetic  
     _2q0 = 2.0f * q0;  
     _2q1 = 2.0f * q1;  
     _2q2 = 2.0f * q2;  
     _2q3 = 2.0f * q3;  
     _4q0 = 4.0f * q0;  
     _4q1 = 4.0f * q1;  
     _4q2 = 4.0f * q2;  
     _8q1 = 8.0f * q1;  
     _8q2 = 8.0f * q2;  
     q0q0 = q0 * q0;  
     q1q1 = q1 * q1;  
     q2q2 = q2 * q2;  
     q3q3 = q3 * q3;  
      
      // Gradient decent algorithm corrective step  
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;  
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;  
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;  
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay; 
 
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalise the gradient
      s0 *= recipNorm;  
      s1 *= recipNorm;  
      s2 *= recipNorm;  
      s3 *= recipNorm;  

      
      // Apply feedback step  
       qDot1 -= beta * s0;  
       qDot2 -= beta * s1;  
       qDot3 -= beta * s2;  
       qDot4 -= beta * s3;  
    }  
      
        // Integrate rate of change of quaternion to yield quaternion  
     q0 += qDot1 * (1.0f / sampleFreq);  
     q1 += qDot2 * (1.0f / sampleFreq);  
     q2 += qDot3 * (1.0f / sampleFreq);  
     q3 += qDot4 * (1.0f / sampleFreq);  
      
        // Normalise quaternion  
     recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  
     q0 *= recipNorm;  
     q1 *= recipNorm;  
     q2 *= recipNorm;  
     q3 *= recipNorm;  
}  



  
   

