#include "motor.h"
#include "students.h"

#include <Arduino.h>
Motor::Motor(int phase_pin, int stdby_pin, int pwm_pin)
:
  d_phase_pin(phase_pin),
  d_stdby_pin(stdby_pin),
  d_pwm_pin(pwm_pin)
{
  pinMode(phase_pin, OUTPUT);
  pinMode(stdby_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  analogWriteFrequency(pwm_pin, 10000);
  analogWriteResolution(12);
}

void Motor::update(float throttle)
{
  digitalWrite(d_stdby_pin, HIGH);
  digitalWrite(d_phase_pin, throttle >= 0 ? LOW : HIGH);
  analogWrite(d_pwm_pin, min(abs(throttle), 4096)); //FIXME MAGIC NR
}
