#include "neuronbot2_imu/neuronbot2_imu.h"

Neuronbot2IMU::Neuronbot2IMU(ros::NodeHandle nh, ros::NodeHandle pnh):
  //  Members default values
  nh_(nh),
  pnh_(pnh),
  use_accelerometer_(true),
  use_gyroscope_(true),
  use_magnetometer_(true),
  use_mag_msg_(false),
  perform_calibration_(true),
  is_calibrated_(false)
{
  pnh_.param<bool>("imu/use_accelerometer", use_accelerometer_, use_accelerometer_);
  pnh_.param<bool>("imu/use_gyroscope", use_gyroscope_, use_gyroscope_);
  pnh_.param<bool>("imu/use_magnetometer", use_magnetometer_, use_magnetometer_);
  pnh_.param<bool>("imu/perform_calibration", perform_calibration_, perform_calibration_);

  raw_sub_ = nh_.subscribe("raw_imu", 5, &Neuronbot2IMU::rawCallback, this);

  imu_cal_srv_ = nh_.advertiseService("imu/calibrate_imu", &Neuronbot2IMU::calibrateCallback, this);

  if (use_accelerometer_ || use_gyroscope_)
  {

    if (!pnh_.getParam("imu/accelerometer_bias", acceleration_bias_) || 
        !pnh_.getParam("imu/gyroscope_bias", gyroscope_bias_))
    {
      ROS_WARN("IMU calibration NOT found.");
      is_calibrated_ = false;
    }
    else
    {
      ROS_INFO("IMU calibration found.");
      pnh_.getParam("imu/accelerometer_bias", acceleration_bias_);
      pnh_.getParam("imu/gyroscope_bias", gyroscope_bias_);
      is_calibrated_ = true;
    }

    pnh_.param<int>("imu/calibration_samples", calibration_samples_, 500);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    pnh_.param<double>("imu/linear_acc_stdev", linear_acc_stdev_, 0.0);
    Neuronbot2IMU::fillRowMajor(linear_acc_covar_, linear_acc_stdev_);

    pnh_.param<double>("imu/angular_vel_stdev", angular_vel_stdev_, 0.0);
    Neuronbot2IMU::fillRowMajor(angular_vel_covar_, angular_vel_stdev_);
  }

  if (use_magnetometer_)
  {

    // Magnetometer calibration values.
    pnh_.param<double>("mag/x/min", mag_x_min_, -0.000078936);
    pnh_.param<double>("mag/x/max", mag_x_max_,  0.000077924);
    pnh_.param<double>("mag/y/min", mag_y_min_, -0.000075532);
    pnh_.param<double>("mag/y/max", mag_y_max_,  0.000076360);
    pnh_.param<double>("mag/z/min", mag_z_min_, -0.000079948);
    pnh_.param<double>("mag/z/max", mag_z_max_,  0.000064216);

    pnh_.param<bool>("imu/use_mag_msg", use_mag_msg_, use_mag_msg_);

    if (use_mag_msg_)
    {
      mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 5);

      pnh_.param<double>("imu/magnetic_field_stdev", magnetic_field_stdev_, 0.0);
      Neuronbot2IMU::fillRowMajor(magnetic_field_covar_, magnetic_field_stdev_);
    }
    else
    {
      mag_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 5);
    }
  }

  ROS_INFO("Starting Raw Imu Bridge.");
}

void Neuronbot2IMU::rawCallback(const neuronbot2_msgs::RawImuConstPtr& raw_msg)
{
  if (!raw_msg->accelerometer && use_accelerometer_)
  {
    ROS_ERROR_ONCE("Accelerometer not found!");
  }
  if (!raw_msg->gyroscope && use_gyroscope_)
  {
    ROS_ERROR_ONCE("Gyroscope not found!");
  }
  if (!raw_msg->magnetometer && use_magnetometer_)
  {
    ROS_ERROR_ONCE("Magnetometer not found!");
  }

  if (perform_calibration_ || !is_calibrated_)
  {
    ROS_WARN_ONCE("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");

    static int taken_samples;

    if (taken_samples < calibration_samples_)
    {
      acceleration_bias_["x"] += raw_msg->raw_linear_acceleration.x;
      acceleration_bias_["y"] += raw_msg->raw_linear_acceleration.y;
      acceleration_bias_["z"] += raw_msg->raw_linear_acceleration.z;

      gyroscope_bias_["x"] += raw_msg->raw_angular_velocity.x;
      gyroscope_bias_["y"] += raw_msg->raw_angular_velocity.y;
      gyroscope_bias_["z"] += raw_msg->raw_angular_velocity.z;
	
      taken_samples++;
    }
    else
    {
      acceleration_bias_["x"] /= calibration_samples_;
      acceleration_bias_["y"] /= calibration_samples_;
      acceleration_bias_["z"] = acceleration_bias_["z"] / calibration_samples_ + GRAVITY;

      gyroscope_bias_["x"] /= calibration_samples_;
      gyroscope_bias_["y"] /= calibration_samples_;
      gyroscope_bias_["z"] /= calibration_samples_;

      ROS_INFO("Calibrating accelerometer and gyroscope complete.");
      ROS_INFO("Bias values can be saved for reuse.");
      ROS_INFO("Accelerometer: x: %f, y: %f, z: %f", acceleration_bias_["x"], acceleration_bias_["y"], acceleration_bias_["z"]);
      ROS_INFO("Gyroscope: x: %f, y: %f, z: %f", gyroscope_bias_["x"], gyroscope_bias_["y"], gyroscope_bias_["z"]);

      pnh_.setParam("imu/accelerometer_bias", acceleration_bias_);
      pnh_.setParam("imu/gyroscope_bias", gyroscope_bias_);

      is_calibrated_ = true;
      perform_calibration_ = false;
      taken_samples = 0;
    }
  }
  else
  {
    if (use_accelerometer_ || use_gyroscope_)
    {
      sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
      imu_msg->header = raw_msg->header;

      imu_msg->angular_velocity.x = raw_msg->raw_angular_velocity.x - gyroscope_bias_["x"];
      imu_msg->angular_velocity.y = raw_msg->raw_angular_velocity.y - gyroscope_bias_["y"];
      imu_msg->angular_velocity.z = raw_msg->raw_angular_velocity.z - gyroscope_bias_["z"];
      imu_msg->orientation_covariance = angular_vel_covar_;

      imu_msg->linear_acceleration.x = raw_msg->raw_linear_acceleration.x - acceleration_bias_["x"];
      imu_msg->linear_acceleration.y = raw_msg->raw_linear_acceleration.y - acceleration_bias_["y"];
      imu_msg->linear_acceleration.z = raw_msg->raw_linear_acceleration.z - acceleration_bias_["z"];
      imu_msg->linear_acceleration_covariance = linear_acc_covar_;

      imu_pub_.publish(imu_msg);
    }

    if (use_magnetometer_)
    {

      if (use_mag_msg_)
      {
        sensor_msgs::MagneticFieldPtr mag_msg = boost::make_shared<sensor_msgs::MagneticField>();
        mag_msg->header = raw_msg->header;

        mag_msg->magnetic_field.x = (raw_msg->raw_magnetic_field.x * MILIGAUSS_TO_TESLA_SCALE - (mag_x_max_ - mag_x_min_) / 2 - mag_x_min_);
        mag_msg->magnetic_field.y = (raw_msg->raw_magnetic_field.y * MILIGAUSS_TO_TESLA_SCALE - (mag_y_max_ - mag_y_min_) / 2 - mag_y_min_);
        mag_msg->magnetic_field.z = (raw_msg->raw_magnetic_field.z * MILIGAUSS_TO_TESLA_SCALE - (mag_z_max_ - mag_z_min_) / 2 - mag_z_min_);
        mag_msg->magnetic_field_covariance = magnetic_field_covar_;

        mag_pub_.publish(mag_msg);
      }
      else
      {
        geometry_msgs::Vector3StampedPtr mag_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();;
        mag_msg->header = raw_msg->header;
        
        mag_msg->vector.x = (raw_msg->raw_magnetic_field.x - (mag_x_max_ - mag_x_min_) / 2 - mag_x_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg->vector.y = (raw_msg->raw_magnetic_field.y - (mag_y_max_ - mag_y_min_) / 2 - mag_y_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg->vector.z = (raw_msg->raw_magnetic_field.z - (mag_z_max_ - mag_z_min_) / 2 - mag_z_min_) * MILIGAUSS_TO_TESLA_SCALE;

        mag_pub_.publish(mag_msg);
      }
    }
  }
}

bool Neuronbot2IMU::calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_WARN("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");
  perform_calibration_ = true;
  return true;
}

void Neuronbot2IMU::fillRowMajor(boost::array<double, 9> & covar, double stdev)
{
  std::fill(covar.begin(), covar.end(), 0.0);
  covar[0] = pow(stdev, 2);  // X(roll)
  covar[4] = pow(stdev, 2);  // Y(pitch)
  covar[8] = pow(stdev, 2);  // Z(yaw)
}
