////////////////////////////////////////////////////////////////////////////////
/// File:     PwmSpeedController.hpp
/// Author:   David Stalter
///
/// Details:  Simple class to control a PWM speed controller.  Based on
///           https://github.com/FRC4014/SRTester/blob/master/SRTester.ino
///           The Servo library is leveraged to interact with the controller,
///           since both are PWM based and there is no other native library.
///
/// Copyright (c) 2019 David Stalter
////////////////////////////////////////////////////////////////////////////////

#ifndef PWMSPEEDCONTROLLER_HPP
#define PWMSPEEDCONTROLLER_HPP

// INCLUDES
#include <Servo.h>                    // for built in Arduino Servo functionality


////////////////////////////////////////////////////////////////////////////////
/// Class:  PwmSpeedController
///
/// Details:  A class for interacting with a PWM based speed controller such as
///           a Talon SR or SparkFun motor controller.
////////////////////////////////////////////////////////////////////////////////
class PwmSpeedController
{
public:
  // Constructor
  PwmSpeedController(int pin) : m_OutputPin(pin)
  {
    m_PwmControl.attach(pin);
  }

  // Update output speed
  inline void SetSpeed(int value)
  {
    const int MAX_VALUE = 100;
    const int MIN_VALUE = -100;
    if (value > MAX_VALUE)
    {
      value = MAX_VALUE;
    }
    else if (value < MIN_VALUE)
    {
      value = MIN_VALUE;
    }
    else
    {
    }

    // 1000us = full reverse
    // 1500us = neutral
    // 2000us = full forward
    const int PWM_SCALE_FACTOR = 5;
    const int PWM_BASE_VALUE = 1500;
    
    // Scale up to 1000-2000
    value = (value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE;
    m_PwmControl.writeMicroseconds(value);
  }
    
private:
  Servo m_PwmControl;
  int m_OutputPin;
};

#endif // PWMSPEEDCONTROLLER_HPP

