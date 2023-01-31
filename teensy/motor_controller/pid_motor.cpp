#include "pid_motor.h"
#include "students.h"
extern ros::NodeHandle* node_handle;


PIDMotor::PIDMotor(char const* name, int stdby_pin, int phase_pin, int encoder_a_pin, int encoder_b_pin, int pwm_pin)
:
  d_motor(phase_pin, stdby_pin, pwm_pin),
  d_encoder(encoder_a_pin, encoder_b_pin),
  d_target_velocity(0),
  d_kp(0),
  d_ki(0),
  d_kd(0),
  d_previous_encoder_position(0),
  d_ticks_per_rotation(1200),
  d_last_delta_rotation(0),

  d_velocity_msg(),
  d_target_velocity_msg(),
  d_velocity_publisher(name, &d_velocity_msg)
{  
  node_handle->advertise(d_velocity_publisher);
}

void PIDMotor::update(float delta_time)
{
  float throttle = pid_update(delta_time) * -1;
  
  //if (abs(d_target_velocity) < 0.0001)
  //  throttle = 0;
  d_motor.update(-throttle);
  //d_motor.update(500);
}


float PIDMotor::pid_update(float delta_time)
{
  float current_velocity = velocity(delta_time);
  return calculate_pid(d_kp, d_ki, d_kd, d_target_velocity, current_velocity, delta_time, d_previous_error, d_integral);
}

float PIDMotor::velocity(float delta_time)
{
  long new_encoder_position = d_encoder.read();
  double velocity = 0;
  calculate_wheel_movement((double)(new_encoder_position - d_previous_encoder_position), d_ticks_per_rotation, delta_time, d_last_delta_rotation, velocity);
  d_previous_encoder_position = new_encoder_position;
  d_velocity_msg.data = velocity;
  d_velocity_publisher.publish(&d_velocity_msg);
  return velocity;
}
