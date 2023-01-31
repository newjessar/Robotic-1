#ifndef INCLUDED_MOTOR_H_
#define INCLUDED_MOTOR_H_

class Motor
{
  int d_phase_pin;
  int d_stdby_pin;
  int d_pwm_pin;
  
public:

  Motor(int phase_pin, int stdby_pin, int pwm_pin);
  void update(float throttle);
};
#endif
