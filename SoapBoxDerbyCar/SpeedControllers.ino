////////////////////////////////////////////////////////////////////////////////
/// File:     SpeedControllers.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic for updating speed controllers present
///           on a soap box derby car.
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

// INCLUDES
#include "SoapBoxDerbyCar.hpp"        // for constants and function declarations

// STATIC DATA
// (none)

// GLOBALS
// (none)


////////////////////////////////////////////////////////////////////////////////
/// Method: SetSteeringDirection
///
/// Details:  Sets the current steering direction (left or right) based on the
///           input value.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::SetSteeringDirection(int value)
{
  if (value > 0)
  {
    m_SteeringDirection = RIGHT;
  }
  else if (value < 0)
  {
    m_SteeringDirection = LEFT;
  }
  else
  {
    m_SteeringDirection = NONE;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: SetSteeringSpeedControllerValue
///
/// Details:  Sets the value of the steering speed controller.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::SetSteeringSpeedControllerValue(int value)
{
  // Just in case it wasn't called elsewhere
  ReadLimitSwitches();
    
  // If the value is negative, steering is going
  // left.  Make sure the limit switch isn't tripped.
  if ((value < 0) && (m_LeftSteeringLimitSwitchValue == 1))
  {
    value = OFF;
  }
  // If the value is positive, steering is going
  // right.  Make sure the limit switch isn't tripped.
  else if ((value > 0) && (m_RightSteeringLimitSwitchValue == 1))
  {
    value = OFF;
  }
  else
  {
  }

  // Update the speed controller and direction
  m_pSteeringSpeedController->SetSpeed(value);
  SetSteeringDirection(value);
  m_CurrentSteeringValue = value;
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


////////////////////////////////////////////////////////////////////////////////
/// Method: ApplyBrake
///
/// Details:  Applies the soap box derby car brake.
///
/// Note: This function can potentially BLOCK until a limit switch is tripped if
///       bWaitForDone is true.  If bWaitForDone is false, this function MUST be
///       called continuously to make sure the brake motor is shut off.  Use
///       this function with caution to not prevent other system critical
///       operations from running and to not overrun the motor!
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ApplyBrake(bool bWaitForDone)
{
  // The limit switches are also tied to interrupt pins.
  // The car be configured to use the interrupts, polling
  // of the pins, or both.  Check Sensors.ino for more info.
  
  if (m_BrakeApplyLimitSwitchValue != 1)
  {
    m_pBrakeSpeedController->SetSpeed(APPLY_BRAKE_PERCENTAGE);
    m_bApplyingBrake = true;
    
    // If requested to wait for the brake to be fully applied
    if (bWaitForDone)
    {
      // Then wait until the limit switch trips
      while (m_BrakeApplyLimitSwitchValue != 1)
      {
        ReadLimitSwitches();
      }
      
      // Motor back off
      m_pBrakeSpeedController->SetSpeed(OFF);
      m_bApplyingBrake = false;
    }
  }
  else
  {
    m_pBrakeSpeedController->SetSpeed(OFF);
    m_bApplyingBrake = false;
  }
  
  // Indicate the brake has been applied so calls to release it
  // properly complete.
  m_bBrakeApplied = true;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReleaseBrake
///
/// Details:  Releases the soap box derby car brake.
///
/// Note: This function can potentially BLOCK until a limit switch is tripped if
///       bWaitForDone is true.  If bWaitForDone is false, this function MUST be
///       called continuously to make sure the brake motor is shut off.  Use
///       this function with caution to not prevent other system critical
///       operations from running and to not overrun the motor!
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReleaseBrake(bool bWaitForDone)
{
  // The limit switches are also tied to interrupt pins.
  // The car be configured to use the interrupts, polling
  // of the pins, or both.  Check Sensors.ino for more info.
  
  // A release of the brake should be continuous, checking the limit
  // switch and updating the motor controller accordingly to make sure
  // gravity doesn't back drive it.
  if (m_BrakeReleaseLimitSwitchValue != 1)
  {
    m_pBrakeSpeedController->SetSpeed(RELEASE_BRAKE_PERCENTAGE);
    m_bReleasingBrake = true;
    
    // If requested to wait for the brake to be fully released
    if (bWaitForDone)
    {
      // Then wait until the limit switch trips
      while (m_BrakeReleaseLimitSwitchValue != 1)
      {
        ReadLimitSwitches();
      }
      
      // Motor back off
      m_pBrakeSpeedController->SetSpeed(OFF);
      m_bReleasingBrake = false;
    }
  }
  else
  {
    m_pBrakeSpeedController->SetSpeed(OFF);
    m_bReleasingBrake = false;
  }

  // Indicate the brake has not been applied so calls to apply it
  // properly complete.
  m_bBrakeApplied = false;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: UpdateSpeedControllers
///
/// Details:  Updates all speed controllers based on user input.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::UpdateSpeedControllers()
{
  // Make sure there's actually data to process
  if (m_ControllerChannelInputs[YAW_INPUT_CHANNEL] == 0)
  {
    return;
  }
  
  // Inputs range from ~1000 (min) to ~2000 (max) with neutral at ~1500.
  // Individual controllers will vary, so be sure to measure each one!
  // Subtracting TRUE_NEUTRAL_INPUT (~1500) gives an input range of ~(-500 -> +500)
  // Divide by the input range to give ~(-1 -> 0) or ~(0 -> +1), which collectively is ~(-1 -> +1)
  const int FULL_RIGHT_INPUT = 1986;
  const int FULL_LEFT_INPUT = 1000;
  const int TRUE_NEUTRAL_INPUT = 1490;
  
  // Read and normalize the input value from true neutral (output here is ~ -500 -> +500).
  double scaledDriveInput = static_cast<double>(m_ControllerChannelInputs[YAW_INPUT_CHANNEL] - TRUE_NEUTRAL_INPUT);
  
  // The left and right input ranges might not be equal, so each case is handled independently.
  if (scaledDriveInput < 0.0F)
  {
    // If it's less than zero, input is to the left, use that range (output here is -1 -> 0).
    scaledDriveInput /= (TRUE_NEUTRAL_INPUT - FULL_LEFT_INPUT);
  }
  else if (scaledDriveInput > 0.0F)
  {
    // If it's greater than zero, input is to the right, use that range (output here is 0 -> +1).
    scaledDriveInput /= (FULL_RIGHT_INPUT - TRUE_NEUTRAL_INPUT);
  }
  else
  {
    // Scaled input is equal to zero, which is already what we want.
  }
  
  // Multiply by 100 since a % is expected by the PWM controller class
  int steerOutputValue = static_cast<int>(scaledDriveInput * 100.0F);
  
  // Trim input to meet minimum requirements
  if ((steerOutputValue <= MIN_OUTPUT_PERCENTAGE) && (steerOutputValue >= -MIN_OUTPUT_PERCENTAGE))
  {
    // Not within required control range, no output
    m_pSteeringSpeedController->SetSpeed(OFF);
  }
  else
  {
    // Just in case it wasn't called elsewhere
    ReadLimitSwitches();
    
    // Check the left limit switch
    if ((steerOutputValue < 0) && (m_LeftSteeringLimitSwitchValue == 1))
    {
      steerOutputValue = OFF;
    }

    // Check the right limit switch
    if ((steerOutputValue > 0) && (m_RightSteeringLimitSwitchValue == 1))
    {
      steerOutputValue = OFF;
    }
    
    // Update talon
    m_pSteeringSpeedController->SetSpeed(steerOutputValue);
    SetSteeringDirection(steerOutputValue);
    m_CurrentSteeringValue = steerOutputValue;
  }

  // Update the brake speed controller based on input
  if (m_bBrakeSwitch)
  {
    ReleaseBrake();
  }
  else
  {
    ApplyBrake();
  }
}

