#include "base.h"
#include "MsTimer2.h"  //This library allows for timer interrupts
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>


ros::NodeHandle* node_handle;
Base* base;
static const int update_rate = 20;
long previous_time;


void p_sub_callback(std_msgs::Float32 const& msg)
{
  base->modify_kp(msg.data);
}

void i_sub_callback(std_msgs::Float32 const& msg)
{
  base->modify_ki(msg.data);
}

void d_sub_callback(std_msgs::Float32 const& msg)
{
  base->modify_kd(msg.data);
}

void k_rho_callback(std_msgs::Float32 const& msg)
{
  base->modify_k_rho(msg.data);
}

void k_alpha_callback(std_msgs::Float32 const& msg)
{
  base->modify_k_alpha(msg.data);
}

void k_beta_callback(std_msgs::Float32 const& msg)
{
  base->modify_k_beta(msg.data);
}

void cmd_vel_callback(geometry_msgs::Twist const& msg)
{
  base->velocity_control_target(msg.linear.x, msg.angular.z);
}

void pos_command_callback(geometry_msgs::Pose2D const& msg)
{
  base->position_control_target(msg.x, msg.y, msg.theta);
}

ros::Subscriber<std_msgs::Float32> p_sub("pid/kp", p_sub_callback);
ros::Subscriber<std_msgs::Float32> i_sub("pid/ki", i_sub_callback);
ros::Subscriber<std_msgs::Float32> d_sub("pid/kd", d_sub_callback);

ros::Subscriber<std_msgs::Float32> krho_sub("pos_params/k_rho", k_rho_callback);
ros::Subscriber<std_msgs::Float32> kalpha_sub("pos_params/k_alpha", k_alpha_callback);
ros::Subscriber<std_msgs::Float32> kbeta_sub("pos_params/k_beta", k_beta_callback);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
ros::Subscriber<geometry_msgs::Pose2D> position_sub("pos_cmd", pos_command_callback);


void setup() {
  // put your setup code here, to run once:
  node_handle = new ros::NodeHandle();
  node_handle->initNode();
  for(;!node_handle->connected(); node_handle->spinOnce());
  
  node_handle->subscribe(p_sub);
  node_handle->subscribe(i_sub);
  node_handle->subscribe(d_sub);
  node_handle->subscribe(krho_sub);
  node_handle->subscribe(kalpha_sub);
  node_handle->subscribe(kbeta_sub);
  node_handle->subscribe(cmd_vel_sub);
  node_handle->subscribe(position_sub);
  //update_pid_params();
  
  previous_time = millis();
  base = new Base();
  MsTimer2::set(update_rate, timed_callback); 
  delay(500);
  MsTimer2::start();
}


void loop() {
  // put your main code here, to run repeatedly:
  node_handle->spinOnce();

  delay(5);
}

void timed_callback()
{
  long new_time = millis();
  float delta_time = (double)(new_time - previous_time) / 1000;
  base->update(delta_time); 

  previous_time = new_time;
}
