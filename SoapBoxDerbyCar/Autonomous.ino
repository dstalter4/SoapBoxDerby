////////////////////////////////////////////////////////////////////////////////
/// File:     Autonomous.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for autonomous operations on
///           a soap box derby car.
///
/// Edit History:
/// - dts 19-OCT-2017 Documentation and headers added.
/// - dts 16-DEC-2017 Enable autonomous logic based on switch state.
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

// INCLUDES
#include "SoapBoxDerbyCar.hpp"        // for constants and function declarations

// STATIC DATA
// (none)

// GLOBALS
// (none)


////////////////////////////////////////////////////////////////////////////////
/// Method: IsAutonomousSwitchSet
///
/// Details:  Checks the autonomous switch digital input and returns its state.
////////////////////////////////////////////////////////////////////////////////
bool SoapBoxDerbyCar::IsAutonomousSwitchSet()
{
  return (digitalRead(AUTONOMOUS_SWITCH_PIN) == 1);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: AutonomousRoutine
///
/// Details:  Primary soap box derby car autonomous routine.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::AutonomousRoutine()
{
  Serial.println("Autonomous executing...");

  // Indicate autonomous is executing, in case any logic
  // elsewhere with the sensors/motor controllers cares.
  m_bIsAutonomousExecuting = true;
  digitalWrite(AUTONOMOUS_LED_PIN, HIGH);
  
  // Reset the hall sensor encoders for this autonomous run
  ResetHallSensorCounts();
  
  // Execute only for as long as autonomous is allowed
  unsigned long autonomousStartTimeMs = GetTimeStampMs();
  while (CalcDeltaTimeMs(autonomousStartTimeMs) < AUTO_MAX_LENGTH_MS)
  {
    // First make sure the switch still says autonomous
    if (!IsAutonomousSwitchSet())
    {
      Serial.println("Autonomous cancelled while running...");
      break;
    }
    
    // Release the brake.  This MUST be done continuously
    // in autonomous to compensate the back drive and make
    // sure the brake speed controller is not always left on.
    ReleaseBrake();

    // Read sensors autonomous will need and use.
    // Hall sensors are interrupt driven.
    ReadLimitSwitches();
    ReadPotentiometers();
    
    // PID to try and control driving.
    
    // If the left side is reading ahead, the car is drifting
    // right and we need to turn back left.
    if (m_LeftHallCount >= (m_RightHallCount + AUTO_HALL_SENSOR_COUNT_MAX_DIFF))
    {
      SetSteeringSpeedControllerValue(AUTO_TURN_LEFT_SPEED);
    }
    // If the right side is reading ahead, the car is drifting
    // left and we need to turn back right.
    else if (m_RightHallCount >= (m_LeftHallCount + AUTO_HALL_SENSOR_COUNT_MAX_DIFF))
    {
      SetSteeringSpeedControllerValue(AUTO_TURN_RIGHT_SPEED);
    }
    // Tracking equally, no steering adjustment needed.
    else
    {
      CenterSteeringByPotentiometer();
    }
  } // End main autonomous while loop

  // Autonomous time expired, stop the motors
  ApplyBrake();
  SetSteeringSpeedControllerValue(OFF);
  
  // Reset the Hall sensor conters in case mode
  // changes to remote control
  ResetHallSensorCounts();
  
  // Autonomous is no longer executing
  m_bIsAutonomousExecuting = false;
  digitalWrite(AUTONOMOUS_LED_PIN, LOW);
  
  Serial.println("Autonomous done executing...");
  
  // Let autonomous only execute once until the
  // switch is flipped back to manual control
  while (IsAutonomousSwitchSet())
  {
  }

  Serial.println("Entering manual control from auto...");
}

