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
SoapBoxDerbyCar *                   SoapBoxDerbyCar::m_pSoapBoxDerbyCar               = nullptr;
SoapBoxDerbyCar::NonVolatileCarData SoapBoxDerbyCar::m_NonVolatileCarData             = {0, 0, false, false, 0};
SoapBoxDerbyCar::DataLogEntry       SoapBoxDerbyCar::m_DataLog[MAX_DATA_LOG_ENTRIES]  = {};
const String                        SoapBoxDerbyCar::SERIAL_PORT_DATA_REQUEST_STRING  = "pi";
const String                        SoapBoxDerbyCar::NON_VOLATILE_CAR_DATA_HEADER     = "SBDC";

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
  Serial.println(F("Entering run..."));

  // Even though loop() is already inside a while (true), this
  // provides an easy way to display a message once before entering run.
  while (true)
  {
    SoapBoxDerbyCar::GetSingletonInstance()->Run();
  }
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
  m_pSteeringSpeedController(new PwmSpeedController(STEERING_SPEED_CONTROLLER_PIN)),
  m_SteeringDirection(NONE),
  m_CurrentSteeringValue(0),
  m_bBrakeApplied(false),
  m_SteeringEncoderValue(0),
  m_SteeringEncoderMultiplier(0),
  m_LeftHallCount(0),
  m_RightHallCount(0),
  m_LeftWheelDistanceInches(0.0),
  m_RightWheelDistanceInches(0.0),
  m_LeftSteeringLimitSwitchValue(0),
  m_RightSteeringLimitSwitchValue(0),
  m_FrontAxlePotentiometerValue(0),
  m_FrontAxlePotMaxLeftValue(0),
  m_FrontAxlePotMaxRightValue(0),
  m_FrontAxlePotCenterValue(0),
  m_LastGoodPotValue(0),
  m_SonarDistanceInches(0),
  m_pDataTransmitSerialPort(&Serial3),
  m_bCalibrationComplete(false),
  m_bStatusLedState(false),
  m_StatusLedTimeStampMs(0UL)
{
  // Give a visual indication that initialization is in progress
  digitalWrite(INITIALIZING_LED_PIN, HIGH);
  
  // Initialize the RAM copy of the non-volatile car data
  NonVolatileCarData eepromCarData;
  GetEepromCarData(eepromCarData);

  memcpy(&m_NonVolatileCarData.m_Header, &NON_VOLATILE_CAR_DATA_HEADER, sizeof(m_NonVolatileCarData.m_Header));
  m_NonVolatileCarData.m_Incarnation = eepromCarData.m_Incarnation + 1;
  
  // Clear out the data log
  ClearDataLog(RAM_LOG);
  
  // Configure serial ports (including default print console)
  ConfigureSerialPorts();
  
  // Configure pin modes
  ConfigureController();
  ConfigureSensors();
  ConfigureDebugPins();

  // Arm the brake
  ArmBrake();

  // Center the steering
  CalibrateSteeringPotentiometer();
  
  // Give a visual indication that initialization is complete
  digitalWrite(INITIALIZING_LED_PIN, LOW);
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
    // In case the manual control LED was on
    digitalWrite(MANUAL_CONTROL_LED_PIN, LOW);
      
    Serial.println(F("Auto waiting..."));

    // In case we came from manual control, disable motors
    SetSteeringSpeedControllerValue(OFF);
    
    // Wait until the controller tells auto to start
    while (!m_bMasterEnable)
    {
      // Update the status light
      BlinkStatusLight();
      
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
      Serial.println(F("Entering autonomous..."));
      
      // Input to start was given
      AutonomousRoutine();

      Serial.println(F("Exiting autonomous..."));
    }
    else
    {
      Serial.println(F("Auto cancelled..."));
    }
  }
  else
  {
    // Update the status light
    BlinkStatusLight();
    
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
      // Visual indication of state
      digitalWrite(MANUAL_CONTROL_LED_PIN, HIGH);
      
      // Update with the user control for steering
      UpdateSpeedControllers();

      // Update the state of the brake based on user input
      UpdateBrakeControl();
    }
    else
    {
      // Visual indication of state
      digitalWrite(MANUAL_CONTROL_LED_PIN, LOW);
    }

    // Log a data entry
    LogData();
  
    // Check if serial data should be sent and transmit it if appropriate
    if (IsCarDataRequested())// || IsSerialTransmitSwitchSet())
    {
      SendCarSerialData();
    }
  
    // Display values to the serial console, if enabled
    if (DEBUG_PRINTS)
    {
      DisplayValues();
    }

    // Get debug commands from the serial console, if enabled
    if (DEBUG_COMMANDS)
    {
      ReadSerialInput();
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: EmergencyStop
///
/// Details:  Emergency stop of the soap box derby car.  Configures each speed
///           controller with a value that will cause the car to enter a safe,
///           known state.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::EmergencyStop(SoapBoxDerbyCar * pInstance)
{
  // Emergency stop will ignore steering and unconditionally apply the brake
  pInstance->m_SteeringDirection = NONE;
  pInstance->m_pSteeringSpeedController->SetSpeed(OFF);
  
  pInstance->ApplyBrake();
}

