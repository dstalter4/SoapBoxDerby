////////////////////////////////////////////////////////////////////////////////
/// File:     Sensors.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for sensors on a soap box
///           derby car.
///
/// Edit History:
/// - dts 19-OCT-2017 Documentation and headers added.
/// - dts 16-DEC-2017 Add ReadPotentiometers().
/// - dts 28-DEC-2017 Switched to SoapBoxDerbyCar class based approach, update
///                   copyright and add GPL header.  Moved encoder and
///                   potentiometer functionality to their own files.
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
/// Method: ConfigureSensors
///
/// Details:  Sets up the pin mode for all sensors.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ConfigureSensors()
{
  // Configure the sensor pin modes
  pinMode(AUTONOMOUS_SWITCH_PIN, INPUT);
  
  pinMode(STEERING_ENCODER_PIN, INPUT);

  pinMode(LEFT_LIMIT_SWITCH_PIN, INPUT);
  pinMode(RIGHT_LIMIT_SWITCH_PIN, INPUT);
  pinMode(BRAKE_RELEASE_LIMIT_SWITCH_PIN, INPUT);
  pinMode(BRAKE_APPLY_LIMIT_SWITCH_PIN, INPUT);
  
  // Configure the Hall sensors for polling OR interrupt mode, not both
  //pinMode(LEFT_HALL_SENSOR_PIN, INPUT);
  //pinMode(RIGHT_HALL_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_HALL_SENSOR_PIN), LeftHallSensorInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_SENSOR_PIN), RightHallSensorInterruptHandler, CHANGE);
  
  pinMode(SONAR_TRIGGER_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadLimitSwitches
///
/// Details:  Reads and stores all limit switch values.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadLimitSwitches()
{
  m_LeftLimitSwitchValue = digitalRead(LEFT_LIMIT_SWITCH_PIN);
  m_RightLimitSwitchValue = digitalRead(RIGHT_LIMIT_SWITCH_PIN);
  m_BrakeReleaseLimitSwitchValue = digitalRead(BRAKE_RELEASE_LIMIT_SWITCH_PIN);
  m_BrakeApplyLimitSwitchValue = digitalRead(BRAKE_APPLY_LIMIT_SWITCH_PIN);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadSonarSensors
///
/// Details:  Reads and stores all sonar values.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadSonarSensors()
{
  const int SONAR_TRIGGER_INIT_DELAY_US   = 2;
  const int SONAR_TRIGGER_PULSE_DELAY_US  = 5;
  const int SONAR_CONVERT_PULSE_TO_INCHES = 74 / 2;
  
  // Prepare output pin with a clean low signal
  digitalWrite(SONAR_TRIGGER_PIN, LOW);
  delayMicroseconds(SONAR_TRIGGER_INIT_DELAY_US);
  
  // Trigger the pin to start the output sonar signal
  digitalWrite(SONAR_TRIGGER_PIN, HIGH);
  delayMicroseconds(SONAR_TRIGGER_PULSE_DELAY_US);
  digitalWrite(SONAR_TRIGGER_PIN, LOW);

  // Listen for a response
  // Division constants from the sample 'Ping' program
  m_SonarDistanceInches = pulseIn(SONAR_ECHO_PIN, HIGH, PULSE_IN_TIMEOUT_US) / SONAR_CONVERT_PULSE_TO_INCHES;
}

