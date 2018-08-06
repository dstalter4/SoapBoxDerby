////////////////////////////////////////////////////////////////////////////////
/// File:     Brake.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic for controlling the brake on a soap box
///           derby car.
///
/// Edit History:
/// - dts 04-AUG-2018 Created.
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
/// Method: ApplyBrake
///
/// Details:  Applies the soap box derby car brake.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ApplyBrake()
{
  // Turn off the relay to the magnet to drop the brake
  digitalWrite(BRAKE_MAGNET_RELAY_PIN, LOW);

  // Give a visual indication of the magnetic field state
  digitalWrite(BRAKE_MAGNET_RELAY_LED_PIN, LOW);
  
  // Indicate the brake has been applied so calls to release it
  // properly complete.
  m_bBrakeApplied = true;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ArmBrake
///
/// Details:  Arms the soap box derby car brake by turning on the magnet.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ArmBrake()
{
  // Turn the relay on to apply a magnetic field and hold the brake up
  digitalWrite(BRAKE_MAGNET_RELAY_PIN, HIGH);

  // Give a visual indication of the magnetic field state
  digitalWrite(BRAKE_MAGNET_RELAY_LED_PIN, HIGH);
  
  // Indicate the brake has not been applied so calls to apply it
  // properly complete.
  m_bBrakeApplied = false;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: UpdateBrakeControl
///
/// Details:  Checks the state of the user input for brake control and updates
///           the state of the brake accordingly.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::UpdateBrakeControl()
{
  if (m_bBrakeSwitch)
  {
    ApplyBrake();
  }
  else
  {
    ArmBrake();
  }
}

