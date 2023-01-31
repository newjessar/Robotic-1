#include "base.h"
#include "students.h"


static const int left_stdby_pin = 28;
static const int left_phase_pin = 29;
static const int left_pwm_pin = 30;
static const int left_enc_a_pin = 27;
static const int left_enc_b_pin = 26;

static const int right_stdby_pin = 37;
static const int right_phase_pin = 36;
static const int right_pwm_pin = 35;
static const int right_enc_a_pin = 39;
static const int right_enc_b_pin = 38;

extern ros::NodeHandle* node_handle;

Base::Base()
:
  d_left_motor("/left_motor", left_stdby_pin, left_phase_pin, left_enc_a_pin, left_enc_b_pin, left_pwm_pin),
  d_right_motor("/right_motor", right_stdby_pin, right_phase_pin, right_enc_a_pin, right_enc_b_pin, right_pwm_pin),
  d_wheel_radius(0.03),
  d_wheel_distance(0.108),
  d_target_x_velocity(0),
  d_target_yaw_velocity(0),
  d_state(VELOCITY_CONTROL),
  d_left_target_publisher("left_target_velocity", &d_float32_msg),
  d_right_target_publisher("right_target_velocity", &d_float32_msg),
  d_x_target_publisher("x_target", &d_float32_msg),
  d_y_target_publisher("y_target", &d_float32_msg),
  d_yaw_target_publisher("yaw_target", &d_float32_msg),
  d_debug_publisher("debug", &d_float32_msg),
  d_pose_publisher("pose_control_pose", &d_pose_msg)
{
  double kp, ki, kd;
  set_parameters(kp, ki, kd, d_k_rho, d_k_alpha, d_k_beta);
  d_left_motor.kp(kp);
  d_right_motor.kp(kp);
  d_left_motor.ki(ki);
  d_right_motor.ki(ki);
  d_left_motor.kd(kd);
  d_right_motor.kd(kd);
  node_handle->advertise(d_left_target_publisher);
  node_handle->advertise(d_right_target_publisher);
  node_handle->advertise(d_x_target_publisher);
  node_handle->advertise(d_y_target_publisher);
  node_handle->advertise(d_yaw_target_publisher);
  node_handle->advertise(d_debug_publisher);
  node_handle->advertise(d_pose_publisher);

}

void Base::target_velocity(float x_velocity, float yaw_velocity)
{
  double left_wheel_target_velocity, right_wheel_target_velocity;
  base_velocity_to_wheel_velocity(x_velocity, yaw_velocity, d_wheel_radius, d_wheel_distance, left_wheel_target_velocity, right_wheel_target_velocity);
  d_left_motor.target_velocity(left_wheel_target_velocity);
  d_right_motor.target_velocity(right_wheel_target_velocity);
  d_float32_msg.data = left_wheel_target_velocity;
  d_left_target_publisher.publish(&d_float32_msg);
  d_float32_msg.data = right_wheel_target_velocity;
  d_right_target_publisher.publish(&d_float32_msg);
}

void Base::velocity_control_target(float x_velocity, float yaw_velocity)
{
  if (d_state == VELOCITY_CONTROL)
    target_velocity(x_velocity, yaw_velocity);
}

void Base::position_control_target(float x, float y, float yaw)
{
  double distance = sqrt(x * x + y * y);
  double angle = atan2(y, x);
  angle += 3.14;
  angle += -1 * yaw;
  d_position_target_x = distance * cos(angle);
  d_position_target_y = distance * sin(angle);
  d_position_target_yaw = -1 * yaw;
  
  d_state = POSITION_CONTROL;
}

void Base::update(float delta_time)
{
  if (d_state == POSITION_CONTROL)
    position_control_update();
  d_left_motor.update(delta_time);
  d_right_motor.update(delta_time);
}

void Base::modify_kp(float p)
{
  d_left_motor.kp(p);
  d_right_motor.kp(p);
}

void Base::modify_ki(float i)
{
  d_left_motor.ki(i);
  d_right_motor.ki(i);
}

void Base::modify_kd(float d)
{
  d_left_motor.kd(d);
  d_right_motor.kd(d);
}

void Base::modify_k_rho(float k_rho)
{
  d_k_rho = k_rho;
}

void Base::modify_k_alpha(float k_alpha)
{
  d_k_alpha = k_alpha;
}

void Base::modify_k_beta(float k_beta)
{
  d_k_beta = k_beta;
}


void Base::position_control_update()
{
  float delta_left = d_left_motor.delta_rotation();
  float delta_right = d_right_motor.delta_rotation();
  double forward_velocity = 0;
  double rotation_velocity = 0;
  bool arrived = linear_position_control(delta_left, delta_right, d_wheel_radius, d_wheel_distance, d_k_rho, d_k_alpha, d_k_beta, d_position_target_x, d_position_target_y, d_position_target_yaw, forward_velocity, rotation_velocity);
  target_velocity(forward_velocity, rotation_velocity);
  d_pose_msg.x = d_position_target_x;
  d_pose_msg.y = d_position_target_y;
  d_pose_msg.theta = d_position_target_yaw;
  d_pose_publisher.publish(&d_pose_msg);
  if (arrived)
  {
    d_state = VELOCITY_CONTROL;
    target_velocity(0,0);
  }
 
}
