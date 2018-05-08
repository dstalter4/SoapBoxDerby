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
  if ((value < 0) && (m_LeftLimitSwitchValue == 1))
  {
    value = OFF;
  }
  // If the value is positive, steering is going
  // right.  Make sure the limit switch isn't tripped.
  else if ((value > 0) && (m_RightLimitSwitchValue == 1))
  {
    value = OFF;
  }
  else
  {
  }

  // Update the speed controller and direction
  m_pSteeringTalon->SetSpeed(value);
  SetSteeringDirection(value);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: EmergencyStop
///
/// Details:  Emergency stop of the soap box derby car.  Configures each speed
///           controller with a value that will cause the car to enter a safe,
///           known state.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::EmergencyStop()
{
  // Emergency stop will ignore steering and unconditionally apply the brake
  m_SteeringDirection = NONE;
  m_pSteeringTalon->SetSpeed(OFF);
  
  ApplyBrake();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ApplyBrake
///
/// Details:  Applies the soap box derby car brake.
///
/// Note: This function BLOCKS for the configured time.  Be careful when using
///       it and adjust the APPLY_BRAKE_TIME_MS constant accordingly.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ApplyBrake()
{
  // TODO: Tie into second brake limit switch.
  
  // If the brake hasn't been applied yet
  if (!m_bBrakeApplied)
  {
    // Start a timer.  Applying the brake works by driving the brake motor
    // for a certain period of time.
    unsigned long startTime = millis();
    while ((millis() - startTime) < APPLY_BRAKE_TIME_MS)
    {
      m_pBrakingTalon->SetSpeed(APPLY_BRAKE_PERCENTAGE);
    }

    // Shut the motor off
    m_pBrakingTalon->SetSpeed(OFF);

    // Indicate the brake was applied
    m_bBrakeApplied = true;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReleaseBrake
///
/// Details:  Releases the soap box derby car brake.
///
/// Note: This function MUST be called repeatedly to keep checking the limit
///       switch and to properly control the speed controller.
///       For manual control, it will be called from UpdateSpeedControllers.
///       For autonomous routines, the logic needs to call it in a loop.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReleaseBrake()
{  
  // A release of the brake should be continuous, checking the limit
  // switch and updating the motor controller accordingly to make sure
  // gravity doesn't back drive it.
  if (m_BrakeReleaseLimitSwitchValue != 1)
  {
    m_pBrakingTalon->SetSpeed(RELEASE_BRAKE_PERCENTAGE);
  }
  else
  {
    m_pBrakingTalon->SetSpeed(OFF);
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
    m_pSteeringTalon->SetSpeed(OFF);
  }
  else
  {
    // Just in case it wasn't called elsewhere
    ReadLimitSwitches();
    
    // Check the left limit switch
    if ((steerOutputValue < 0) && (m_LeftLimitSwitchValue == 1))
    {
      steerOutputValue = OFF;
    }

    // Check the right limit switch
    if ((steerOutputValue > 0) && (m_RightLimitSwitchValue == 1))
    {
      steerOutputValue = OFF;
    }
    
    // Update talon
    m_pSteeringTalon->SetSpeed(steerOutputValue);
    SetSteeringDirection(steerOutputValue);
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


////////////////////////////////////////////////////////////////////////////////
/// Method: RampSpeedControllerUpDownTest
///
/// Details:  Test method to demonstrate a speed controller can be properly
///           communicated with.  It goes from full reverse to full forward and
///           back, allowing a user to examine the speed controller LED color
///           to verify behavior.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::RampSpeedControllerUpDownTest(PwmSpeedController * pSpeedController)
{
  // Ramp up from full reverse to full forward
  for (int value = -100; value <= 100; value++)
  {
    pSpeedController->SetSpeed(value);
    delay(TENTH_OF_A_SECOND_DELAY_MS);
  }

  // Ramp down from full forward to full reverse
  for (int value = 100; value >= -100; value--)
  {
    pSpeedController->SetSpeed(value);
    delay(TENTH_OF_A_SECOND_DELAY_MS);
  }
}

