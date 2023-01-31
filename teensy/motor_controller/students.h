#ifndef INCLUDED_STUDENTS_H_
#define INCLUDED_STUDENTS_H_

void set_parameters(double& k_p, double& k_i, double& k_d, double& k_rho, double& k_alpha, double& k_beta);
void base_velocity_to_wheel_velocity(double forward_velocity, double rotational_velocity, double wheel_radius, double track, double& left_wheel_velocity, double& right_wheel_velocity);
void calculate_wheel_movement(double delta_encoder_position, int ticks_per_rotation, double delta_time, float& delta_radians, double& velocity);
double calculate_pid(double kp, double ki, double kd, double target_velocity, double measured_velocity, double delta_time, float& previous_error, float& sum);
bool linear_position_control(double delta_left, double delta_right, double wheel_radius, double track, double k_rho, double k_alpha, double k_beta, float& x, float& y, float& theta, double& v, double& omega);


#endif
