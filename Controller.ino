////////////////////////////////////////////////////////////////////////////////
/// File:     Controller.ino
/// Authoer:  David Stalter
///
/// Details:  Contains the main logic and workflow for interacting with the user
///           input controller with a soap box derby car.
///
/// Note: The input channels are as follows:
///       1. Right stick x-axis (right is + from neutral, left is - from neutral)
///       2. Right stick y-axis (top is + from neutral, bottom is - from neutral)
///       3. Left stick y-axis (top is + from neutral, bottom is - from neutral)
///       4. Left stick x-axis (right is + from neutral, left is - from neutral)
///       5. SWA
///       6. SWD
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
/// Method: ConfigureController
///
/// Details:  Configures the pin modes of the pins used for controller input.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ConfigureController()
{
  // Set pin mode for all controller inputs
  pinMode(CH1_INPUT_PIN, INPUT);
  pinMode(CH2_INPUT_PIN, INPUT);
  pinMode(CH3_INPUT_PIN, INPUT);
  pinMode(CH4_INPUT_PIN, INPUT);
  pinMode(CH5_INPUT_PIN, INPUT);
  pinMode(CH6_INPUT_PIN, INPUT);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: IsControllerOn
///
/// Details:  Checks all controller inputs and returns whether the controller
///           is on.
////////////////////////////////////////////////////////////////////////////////
bool SoapBoxDerbyCar::IsControllerOn()
{
  bool bOn = false;
  
  // Loop over all the input channels
  for (int i = 1; i <= NUM_CONTROLLER_INPUT_CHANNELS; i++)
  {
    // If something sent a non-zero value, controller is on
    if (m_ControllerChannelInputs[i] != 0)
    {
      bOn = true;
      break;
    }
  }
  
  return bOn;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: IsRecalibrationRequested
///
/// Details:  Checks for a recalibration request on the controller.
////////////////////////////////////////////////////////////////////////////////
bool SoapBoxDerbyCar::IsRecalibrationRequested()
{
  const int RECALIBRATE_INPUT_CHANNEL_THRESHOLD = 1100;
  
  // Make sure the controller is on and the input stick was moved sufficiently far
  if ((m_ControllerChannelInputs[RECALIBRATE_INPUT_CHANNEL] != 0)
      &&(m_ControllerChannelInputs[RECALIBRATE_INPUT_CHANNEL] < RECALIBRATE_INPUT_CHANNEL_THRESHOLD))
  {
    return true;
  }
  else
  {
    return false;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadControllerInput
///
/// Details:  Reads and stores all controller input values via pulseIn().
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadControllerInput()
{
  // Pins 2,3,4 are currently not utilized
  m_ControllerChannelInputs[YAW_INPUT_CHANNEL]            = pulseIn(CH1_INPUT_PIN, HIGH, PULSE_IN_TIMEOUT_US);
  //m_ControllerChannelInputs[2]                            = pulseIn(CH2_INPUT_PIN, HIGH, PULSE_IN_TIMEOUT_US);
  //m_ControllerChannelInputs[3]                            = pulseIn(CH3_INPUT_PIN, HIGH, PULSE_IN_TIMEOUT_US);
  m_ControllerChannelInputs[RECALIBRATE_INPUT_CHANNEL]    = pulseIn(CH4_INPUT_PIN, HIGH, PULSE_IN_TIMEOUT_US);
  m_ControllerChannelInputs[BRAKE_INPUT_CHANNEL]          = pulseIn(CH5_INPUT_PIN, HIGH, PULSE_IN_TIMEOUT_US);
  m_ControllerChannelInputs[MASTER_ENABLE_INPUT_CHANNEL]  = pulseIn(CH6_INPUT_PIN, HIGH, PULSE_IN_TIMEOUT_US);
  
  // Inputs range from ~1000 (off) to ~1500 (on).
  // Pick a value approximately in the middle.
  // Individual controllers will vary, so be sure to measure each one!
  const int SWITCH_THRESHHOLD_VALUE = 1350;
  
  // Update the state of the brake switch
  if (m_ControllerChannelInputs[BRAKE_INPUT_CHANNEL] < SWITCH_THRESHHOLD_VALUE)
  {
    m_bBrakeSwitch = false;
  }
  else
  {
    m_bBrakeSwitch = true;
  }

  // Update the state of the master enable
  if (m_ControllerChannelInputs[MASTER_ENABLE_INPUT_CHANNEL] < SWITCH_THRESHHOLD_VALUE)
  {
    m_bMasterEnable = false;
  }
  else
  {
    m_bMasterEnable = true;
  }
}
