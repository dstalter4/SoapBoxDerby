////////////////////////////////////////////////////////////////////////////////
/// File:     DebugUtils.ino
/// Author:   David Stalter
///
/// Details:  Contains debug related functions for a soap box derby car.
///
/// Edit History:
/// - dts 19-OCT-2017 Documentation and headers added.
/// - dts 28-DEC-2017 Switched to SoapBoxDerbyCar class based approach, update
///                   copyright and add GPL header.  Add ProcessAssert().
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

// INCLUDES
#include "SoapBoxDerbyCar.hpp"        // for constants and function declarations

// STATIC DATA
// (none)

// GLOBALS
// (none)


////////////////////////////////////////////////////////////////////////////////
/// Method: ConfigureDebugPins
///
/// Details:  Configures the pin mode of pins used for debug purposes.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ConfigureDebugPins()
{
  pinMode(DEBUG_OUTPUT_1_LED_PIN, OUTPUT);
  pinMode(DEBUG_OUTPUT_2_LED_PIN, OUTPUT);
  pinMode(AUTONOMOUS_LED_PIN, OUTPUT);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ProcessAssert
///
/// Details:  Function called when an ASSERT in the code occurs.  For the soap
///           box derby car, it will cause an emergency stop and strobe the
///           debug LEDs.  This function is fatal and cannot be recovered from
///           without power cycling.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ProcessAssert()
{
  bool ledState = static_cast<bool>(HIGH);
  while (true)
  {
    digitalWrite(DEBUG_OUTPUT_1_LED_PIN, static_cast<int>(ledState));
    digitalWrite(DEBUG_OUTPUT_2_LED_PIN, static_cast<int>(ledState));

    ledState = !ledState;

    if (m_pSoapBoxDerbyCar != nullptr)
    {
      EmergencyStop(m_pSoapBoxDerbyCar);
    }
    
    const int ASSERT_DELAY_MS = 1000;    
    delay(ASSERT_DELAY_MS);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: DisplayValues
///
/// Details:  Displays debug information about the inputs, sensors and other
///           state control variables.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::DisplayValues()
{
  static unsigned long currentTimeStamp = 0;
  static unsigned long oldTimeStamp = 0;

  // Only display things every so often to not hog the CPU
  currentTimeStamp = millis();
  if ((currentTimeStamp - oldTimeStamp) > DEBUG_PRINT_INTERVAL_MS)
  {
    Serial.print("Time: ");
    Serial.println(currentTimeStamp);
    
    Serial.print("Steering input: ");
    Serial.println(m_ControllerChannelInputs[YAW_INPUT_CHANNEL]);
    Serial.print("Brake input: ");
    Serial.println(m_ControllerChannelInputs[BRAKE_INPUT_CHANNEL]);
    Serial.print("Emergency stop input: ");
    Serial.println(m_ControllerChannelInputs[MASTER_ENABLE_INPUT_CHANNEL]);
    Serial.print("Steering encoder: ");
    Serial.println(m_SteeringEncoderValue);
    Serial.print("Steering encoder multiplier: ");
    Serial.println(m_SteeringEncoderMultiplier);
    
    Serial.print("Left hall sensor: ");
    Serial.println(m_LeftHallSensorValue);
    Serial.print("Right hall sensor: ");
    Serial.println(m_RightHallSensorValue);
    Serial.print("Left hall count: ");
    Serial.println(m_LeftHallCount);
    Serial.print("Right hall count: ");
    Serial.println(m_RightHallCount);
    
    Serial.print("Left limit switch: ");
    Serial.println(m_LeftLimitSwitchValue);
    Serial.print("Right limit switch: ");
    Serial.println(m_RightLimitSwitchValue);
    Serial.print("Brake release limit switch: ");
    Serial.println(m_BrakeReleaseLimitSwitchValue);
    Serial.print("Brake apply limit switch: ");
    Serial.println(m_BrakeApplyLimitSwitchValue);
    
    Serial.print("Front axle potentiometer: ");
    Serial.println(m_FrontAxlePotentiometerValue);
    
    Serial.print("Sonar sensor: ");
    Serial.println(m_SonarDistanceInches);
  
    Serial.println();
    Serial.println();

    oldTimeStamp = currentTimeStamp;
  }
}

