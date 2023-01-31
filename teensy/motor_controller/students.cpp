#include "students.h"
#include <Arduino.h>
/*
 * Some small notions regarding C++. The code in this file was made to be as similar to C, while keeping it accessable with some C++ features, namely the usage of references.
 * In earlier courses, which use the C language, you've probably have seen two types of function arguments. Either passing a copy of a variable, or passing a pointer to an variable.
 * E.g: 
 * ***************Code example***************************
 * int add_two(int a)
 * {
 *    a += 2;
 *    return a;
 * }
 * 
 * void add_three(int* a)
 * {
 *    *a += 3;
 * }
 * 
 * int value = 3;
 * int other_value = add_two(value);
 * //other_value is now 5;
 * //value is still 3;
 * add_three(value);
 * //value is now 6;
 * *******************************************************
 * 
 * C++ allows us to do a similar 'return by reference', without the hassle of pointer syntax
 * 
 * **************Code example**************
 * void add_two(int& a)
 * {
 *   a += 3;
 * }
 * 
 * int value = 3; 
 * add_two(value);
 * //value is now 5
 * ****************************************
 * As you can see, variable 'a' behaves just as a regular variable.
 * This technique is oftenly used below to obtain your calculated answers in the skeleton code from the other files.
 * 
 * 
 */

 /*
  * set_parameters:
  *   This function should be used to store your parameters in both for the PID control loop, as well as for position control.
  *   All arguments are used to pass your filled in values to other pieces of code, so you only have to adapt the default value '0.0', as already set in this function.
  */

void set_parameters(double& k_p, double& k_i, double& k_d, double& k_rho, double& k_alpha, double& k_beta)
{
  //set your tuned PID parameters here:
  k_p = 0;
  k_i = 0;
  k_d = 0;

  //store your found position control parameters here:
  k_rho = 0;
  k_alpha = 0;
  k_beta = 0;
}



/* This function should calculate what the rotational velocities of the wheels should be, given a command velocity for the robot itself.
 * 
 * double forward_velocity : This variable contains the target forward velocity of the robot itself (m/s)
 * double rotational_velocity : This variable contains the target rotation velocity of the robot itself. (rads/s)
 * double wheel_radius : The radius of the wheels.
 * double track : The distance between the wheels.
 * double& left_wheel_velocity : Store the calculated rotational velocity for the left wheel in this variable (rads/s)
 * double& right_wheel_velocity : Store the calculated rotational velocity for the right wheel in this variable (rads/s)
 */
void base_velocity_to_wheel_velocity(double forward_velocity, double rotational_velocity, double wheel_radius, double track, double& left_wheel_velocity, double& right_wheel_velocity)
{

}

/*
 * double delta_encoder_position: is the provided result from new_encoder_position - previous_encoder_position;
 * int ticks_per_rotation: The number of encoder ticks per full wheel rotation
 * double delta_time: time since the previous step, in seconds.
 * double& delta_radians: store the result of your calculations of how many radians the wheel turned in this variable.
 * double& velocity: store the result of your calculations of the wheel velocity in rads/s in this variable.
 */
void calculate_wheel_movement(double delta_encoder_position, int ticks_per_rotation, double delta_time, float& delta_radians, double& velocity)
{

}



/* This function should implement the PID formula.
 *  
 *  double kp : the P constant
 *  double ki : the I constant
 *  double kd : the D constant
 *  
 *  double target_velocity : the wheel velocity in rads/s that should be maintained.
 *  double measured_velocity : the actual current wheel velocity (rads/s)
 *  double delta_time : the time passed since the previous update iteration (seconds).
 *  double& previous_error: contains the calculated error from the previous iteration. 
 *    Once you've calculated the new error for this iteration, store it again in the variable at the end of your function.
 *  double& integral : A variable that you can use to make summations over multiple iterations.
 */
double calculate_pid(double kp, double ki, double kd, double target_velocity, double measured_velocity, double delta_time, float& previous_error, float& sum)
{

}





/*
 * This function should implement linear position control. 
 * 
 * This function should return a boolean, indicating whether the robot has reached its target location.
 * If the robot is close enough, and the orientation error is almost zero, the function should return true.
 * Returning true will cause the code to stop with position control, until a next position control command is given.
 * 
 * The arguments are: 
 * double delta_left : The difference in the left wheel position since the last iteration (rads)
 * double delta_right : The difference in the right wheel position since the last iteration (rads)
 * double wheel_radius : The wheel radius (m)
 * double track : The distance between the wheels (m)
 * double& x: The current x-coordinate of the robot, with respect to the target location (m). Note: This variable is a reference. At the end of this function, update this x coordinate.
 * double& y: The current y-coordinate of the robot, with respect to the target location (m). Note: This variable is a reference. At the end of this function, update this y coordinate.
 * double& theta: The current orientation of the robot, with respect to the target location (rads). Note: This variable is a reference. At the end of this function, update this orientation.
 * 
 * double& v: The output forward velocity of the robot that you've calculated (m/s)
 * double& omega: The output rotational velocity of the robot, that you've calculated (rads/s).
 */
bool linear_position_control(double delta_left, double delta_right, double wheel_radius, double track, double k_rho, double k_alpha, double k_beta, float& x, float& y, float& theta, double& v, double& omega)
{
  //double delta_forward = .. // calculate how much the robot moved forward since the last iteration
  //double delta_theta = .. // calculate how much the robot rotated since the last iteration

  //update x,y,theta with this information

  //calculate rho
  //double rho = ...
  //double rho = ...;
  
  //calculate alpha
  //double alpha = ...;
  //calculate beta
  //double beta = ...;

  //define your constants:
  //double k_rho = ...
  //double k_alpha = ...
  //double k_beta = ...

  //calculate the forward target velocity for the robot:
  //double v = ...
  //double omega = ...

  //calculate a scaling factor for the velocities, which can be applied on both v and omega, such that:
  //  - v is constant at 0.15, unless that causes omega to be larger than 1.0.
  //  - if scaling v & omega causes omega > 1.0, calculate a different scaling factor such that omega = 1.0
  
  //double scaling_factor = ....
  
  //apply scaling factor
  //v = v * scaling_factor;
  //omega = omega * scaling_factor;


  //determine whether the robot has reached the destination. If so, return true, otherwise return false;
  return true or false;
}
