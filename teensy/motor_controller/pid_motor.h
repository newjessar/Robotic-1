#ifndef INCLUDED_PID_MOTOR_H_
#define INCLUDED_PID_MOTOR_H_

#include <ros.h>

#include "encoder.h"
#include "motor.h"
#include <std_msgs/Float32.h>

class PIDMotor
{
  
  Motor d_motor;
  Encoder d_encoder;
  float d_target_velocity;
  float d_kp;
  float d_ki;
  float d_kd;
  float d_previous_error;
  float d_integral;
  long d_previous_encoder_position;
  int d_ticks_per_rotation;
  float d_last_delta_rotation;
  std_msgs::Float32 d_velocity_msg;
  std_msgs::Float32 d_target_velocity_msg;
  ros::Publisher d_velocity_publisher;
   
public:
  PIDMotor(char const* name, int stdby_pin, int phase_pin, int encoder_a_pin, int encoder_b_pin, int pwm_pin);
  void target_velocity(float velocity);
  void update(float delta_time);
  float pid_update(float delta_time);
  void kp(float p);
  void ki(float i);
  void kd(float d);
  float delta_rotation();
  friend double calculate_pid(double kp, double ki, double kd, double target_velocity, double measured_velocity, double delta_time, double& previous_error, double& sum);
private:
  float velocity(float delta_time);
  void reset_errors();
};

inline float PIDMotor::delta_rotation()
{
  return d_last_delta_rotation;
}

inline void PIDMotor::reset_errors()
{
  d_previous_error = 0;
  d_integral = 0;
}
inline void PIDMotor::target_velocity(float velocity)
{
  d_target_velocity = velocity;
}

inline void PIDMotor::kp(float p)
{
  d_kp = p;
  reset_errors();
}

inline void PIDMotor::ki(float i)
{
  d_ki = i;
  reset_errors();
}

inline void PIDMotor::kd(float d)
{
  d_kd = d;
  reset_errors();
}
  
#endif
