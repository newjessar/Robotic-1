#ifndef INCLUDED_BASE_H_
#define INCLUDED_BASE_H_

#include "pid_motor.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>



class Base
{
  enum State
  {
    VELOCITY_CONTROL,
    POSITION_CONTROL,
  };
  PIDMotor d_left_motor;
  PIDMotor d_right_motor;
  float d_wheel_radius; //meters
  float d_wheel_distance;
  float d_target_x_velocity;
  float d_target_yaw_velocity;
  float d_position_target_x;
  float d_position_target_y;
  float d_position_target_yaw;
  double d_k_rho;
  double d_k_alpha;
  double d_k_beta;
  State d_state;
  std_msgs::Float32 d_float32_msg;
  geometry_msgs::Pose2D d_pose_msg;
  ros::Publisher d_left_target_publisher;
  ros::Publisher d_right_target_publisher;
  ros::Publisher d_x_target_publisher;
  ros::Publisher d_y_target_publisher;
  ros::Publisher d_yaw_target_publisher;
  ros::Publisher d_debug_publisher;
  ros::Publisher d_pose_publisher;

public:
  Base();
  void velocity_control_target(float x_velocity, float yaw_velocity);
  void position_control_target(float x, float y, float yaw);
  void position_control_update();
  void update(float delta_time);
  void modify_kp(float p);
  void modify_ki(float i);
  void modify_kd(float d);
  void modify_k_rho(float k_rho);
  void modify_k_alpha(float k_alpha);
  void modify_k_beta(float k_beta);
  friend void set_parameters(double& k_p, double& k_i, double& k_d, double& k_rho, double& k_alpha, double& k_beta);
private:
  void target_velocity(float x_velocity, float yaw_velocity);

};
#endif
