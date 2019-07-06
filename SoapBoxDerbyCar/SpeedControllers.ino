////////////////////////////////////////////////////////////////////////////////
/// File:     SpeedControllers.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic for updating speed controllers present
///           on a soap box derby car.
///
/// Copyright (c) 2019 David Stalter
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
}

