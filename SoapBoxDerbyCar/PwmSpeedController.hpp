////////////////////////////////////////////////////////////////////////////////
/// File:     PwmSpeedController.hpp
/// Author:   David Stalter
///
/// Details:  Simple class to control a PWM speed controller.  Based on
///           https://github.com/FRC4014/SRTester/blob/master/SRTester.ino
///           The Servo library is leveraged to interact with the controller,
///           since both are PWM based and there is no other native library.
///
/// Edit History:
/// - dts 19-OCT-2017 Documentation and headers added.
/// - dts 28-DEC-2017 Switched to SoapBoxDerbyCar class based approach, update
///                   copyright and add GPL header.
///
/// Copyright (c) 2018 David Stalter
///
/// This file is part of SoapBoxDerbyCar.
///
/// SoapBoxDerbyCar is free software: you can redistribute it and/or modify it
/// under the terms of the GNU General Public License as published by the Free
/// Software Foundation, either version 3 of the License, or (at your option)
/// any later version.
///
/// SoapBoxDerbyCar is distributed in the hope that it will be useful, but
/// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
/// or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
/// for more details.
///
/// You should have received a copy of the GNU General Public License along with
/// SoapBoxDerbyCar.  If not, see <http://www.gnu.org/licenses/>.
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

