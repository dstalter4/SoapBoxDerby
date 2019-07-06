////////////////////////////////////////////////////////////////////////////////
/// File:     Brake.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic for controlling the brake on a soap box
///           derby car.
///
/// Copyright (c) 2018 David Stalter
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

