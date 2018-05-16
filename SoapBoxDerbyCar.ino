////////////////////////////////////////////////////////////////////////////////
/// File:     SoapBoxDerbyCar.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for a user driven or
///           autonomous soap box derby car.
///
/// Edit History:
/// - dts 19-OCT-2017 Documentation and headers added.
/// - dts 16-DEC-2017 Enable autonomous logic based on switch state.
/// - dts 28-DEC-2017 Switched to SoapBoxDerbyCar singleton approach, update
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
SoapBoxDerbyCar * SoapBoxDerbyCar::m_pSoapBoxDerbyCar = nullptr;

// GLOBALS
// (none)


////////////////////////////////////////////////////////////////////////////////
/// Method: setup
///
/// Details:  The Arduino initialization function called during controller
///           start up. 
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Enable the UART
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  // Create the soap box derby car singleton
  SoapBoxDerbyCar::CreateSingletonInstance();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: loop
///
/// Details:  The continuous Arduino background user loop.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  SoapBoxDerbyCar::GetSingletonInstance()->Run();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: SoapBoxDerbyCar
///
/// Details:  Constructor for a soap box derby car.  Creates and initializes
///           all member variables.
////////////////////////////////////////////////////////////////////////////////
SoapBoxDerbyCar::SoapBoxDerbyCar() :
  m_bIsAutonomousExecuting(false),
  m_ControllerChannelInputs(),
  m_bBrakeSwitch(false),
  m_bMasterEnable(false),
  m_pSteeringTalon(new PwmSpeedController(STEERING_TALON_PIN)),
  m_pBrakingTalon(new PwmSpeedController(BRAKING_TALON_PIN)),
  m_bBrakeApplied(false),
  m_SteeringEncoderValue(0),
  m_SteeringEncoderMultiplier(0),
  m_LeftHallSensorValue(0),
  m_RightHallSensorValue(0),
  m_LeftHallCount(0),
  m_RightHallCount(0),
  m_LeftLimitSwitchValue(0),
  m_RightLimitSwitchValue(0),
  m_BrakeReleaseLimitSwitchValue(0),
  m_BrakeApplyLimitSwitchValue(0),
  m_FrontAxlePotentiometerValue(0),
  m_FrontAxlePotMaxLeftValue(0),
  m_FrontAxlePotMaxRightValue(0),
  m_FrontAxlePotCenterValue(0),
  m_LastGoodPotValue(0),
  m_SonarDistanceInches(0),
  m_SteeringDirection(NONE),
  m_bCalibrationComplete(false)
{
  // Configure pin modes
  ConfigureController();
  ConfigureSensors();
  ConfigureDebugPins();

  // Apply the brake
  //ApplyBrake();

  // Center the steering
  CalibrateSteeringPotentiometer();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: Run
///
/// Details:  Soap box derby car runtime logic.  It checks for manual or
///           autonomous control and executes accordingly.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::Run()
{
  if (IsAutonomousSwitchSet())
  {
    Serial.println("Auto waiting...");

    // In case we came from manual control, disable motors
    ApplyBrake();
    SetSteeringSpeedControllerValue(OFF);
    
    // Wait until the controller tells auto to start
    while (!m_bMasterEnable)
    {
      // Be sure to update the controller values
      ReadControllerInput();
      
      // Check for a request to recalibrate
      if (IsRecalibrationRequested())
      {
        CalibrateSteeringPotentiometer();
      }
      
      // Check for a switch back to manual control
      if (!IsAutonomousSwitchSet())
      {
        break;
      }
    }

    // If we make it here, either autonomous should
    // start or we reverted to manual control.  Make
    // sure autonomous is still intended.
    if (IsAutonomousSwitchSet())
    {
      Serial.println("Entering autonomous...");
      
      // Input to start was given
      AutonomousRoutine();

      Serial.println("Exiting autonomous...");
    }
    else
    {
      Serial.println("Auto cancelled...");
    }
  }
  else
  {
    // Get the latest input values
    ReadControllerInput();
    
    // Read the Hall sensors
    // Note: Currently interrupt driven
    //ReadHallSensors();
  
    // Read the limit switches
    ReadLimitSwitches();

    // Read the potentiometers
    ReadPotentiometers();
  
    // Read the sonar sensors
    //ReadSonarSensors();
  
    // Read the encoders
    //ReadEncoders();
  
    // Make sure the controller is sending data and check the enable switch
    if (IsControllerOn() && m_bMasterEnable)
    {
      // Update with the user control for steering and braking
      UpdateSpeedControllers();
    }
    else
    {
      // No controller communication or enable switch is off,
      // apply the brake and wait for state change
      if (!IsControllerOn())
      {
        // TEMPORARY!  Remove once second limit switch works.
        //ApplyBrake();
      }
    }
  
    // Display values to the serial console, if enabled
    if (DEBUG_PRINTS)
    {
      DisplayValues();
    }
  }
}

