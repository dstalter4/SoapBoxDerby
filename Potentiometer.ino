////////////////////////////////////////////////////////////////////////////////
/// File:     Potentiometer.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for potentiometers on a soap
///           box derby car.
///
/// Edit History:
/// - dts 28-DEC-2017 Created from potentiometer methods in Sensors.ino.
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
/// Method: CenterSteeringByPotentiometer
///
/// Details:  Automatically turns the steering to max left, then to max right,
///           and finally attempts to move back to true center.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::CenterSteeringByPotentiometer()
{
  Serial.println("Centering steering by pot...");
  
  // Calibrate max left
  
  // Move to the left limit switch
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_LEFT_SPEED);
  
  while (m_LeftLimitSwitchValue != 1)
  {
    // Keep updating limit switches and potentiometer
    ReadLimitSwitches();
    ReadPotentiometers();
  }
  
  // Hit left limit switch, motor off
  SetSteeringSpeedControllerValue(OFF);
  
  // Pause to let things settle
  delay(AUTO_CENTERING_CALIBRATION_DELAY_MS);
  
  // Read the potentiometer again to get the last value
  ReadPotentiometers();
  
  // Save off the value
  m_FrontAxlePotMaxLeftValue = m_FrontAxlePotentiometerValue;
  
  // Calibrate max right

  // Move to the right limit switch
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_RIGHT_SPEED);
  
  while (m_RightLimitSwitchValue != 1)
  {
    // Keep updating limit switches and potentiometer
    ReadLimitSwitches();
    ReadPotentiometers();   
  }

  // Hit right limit switch, motor off
  SetSteeringSpeedControllerValue(OFF);
  
  // Pause to let things settle
  delay(AUTO_CENTERING_CALIBRATION_DELAY_MS);
  
  // Read the potentiometer again to get the last value
  ReadPotentiometers();
  
  // Save off the value
  m_FrontAxlePotMaxRightValue = m_FrontAxlePotentiometerValue;
  
  Serial.print("Left calibration pot value: ");
  Serial.println(m_FrontAxlePotMaxLeftValue);
  Serial.print("Right calibration pot value: ");
  Serial.println(m_FrontAxlePotMaxRightValue);
  Serial.print("Pot diff: ");
  Serial.println(m_FrontAxlePotMaxLeftValue  - m_FrontAxlePotMaxRightValue);
  Serial.println("Centering calibration complete...");
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadPotentiometers
///
/// Details:  Reads the state of the potentiometers.
///
/// Wheel axle (left to right) is 32"
/// Wheel base (front to back) is 61"
/// http://www.davdata.nl/math/turning_radius.html
/// r = w / tan(theta) (front wheel radius)
/// R = w / sin(theta) (back wheel radius)
///
/// theta = S/r
/// theta -> from potentiometer
/// r -> calculated from front wheel equation above
/// S -> arclength to adjust
///
/// S inner wheel = (w / tan(theta)) * theta
/// S outer wheel = ((w / tan(theta)) + d axle) * theta
///
/// Wheel diameter is 12 1/8" (12.125)
/// This means wheel circumference C = pi*d = 3.14159265 * 12.125 = 38.0918109"
/// Depending on magnet configuration, this gives out of sync distances of:
///  Magnets | Inches
/// -----------------------
///  1       | 38.0918109"
///  2       | 19.0459055"
///  3       | 12.6972703"
///  4       |  9.5229527"
///  6       |  6.3486351"
///  8       |  4.7614763"
///
/// Potentiometer has input range of 0 -> 1023 and 250 degrees.
/// This gives a default ratio of .244140 degrees per click.
/// Large gear on axle is 84 tooth, small gear is 36 tooth.
/// This gives a 7:3 ratio.
/// .244140 * 7/3 = .569661 degrees/click
/// .244140 * 3/7 = .104632 degrees/click
/// Empirical measurements show a range of ~60 clicks, therefore:
/// Actual axis range of motion is ~14.6484735 degrees
/// Potentiometer range of motion is ~34.1796785 degrees
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadPotentiometers()
{
  m_FrontAxlePotentiometerValue = analogRead(FRONT_AXLE_POTENTIOMETER_PIN);
  
  // Pot is wired up where left -> right is decreasing
  // Find the center value based on calibration routine
  int centerPotPosition = m_FrontAxlePotMaxRightValue + ((m_FrontAxlePotMaxLeftValue - m_FrontAxlePotMaxRightValue) / 2);

  // Find the potentiometer's diff from true center
  int potDiff = 0;
  SteeringDirection axleDirection = NONE;
  if (m_FrontAxlePotentiometerValue > centerPotPosition)
  {
    // Axle angled to the left
    potDiff = m_FrontAxlePotentiometerValue - centerPotPosition;
    axleDirection = LEFT;
  }
  else if (m_FrontAxlePotentiometerValue < centerPotPosition)
  {
    // Axle angled to the right
    potDiff = centerPotPosition - m_FrontAxlePotentiometerValue;
    axleDirection = RIGHT;
  }
  else
  {
    potDiff = 0;
    axleDirection = NONE;
  }
  
  // The potentiometer has a 250 degree range of motion, and 1024 input value
  // GEAR_RATIO comes from the teeth ratio on the gears
  const float POT_INPUT_RATIO       = 250.0 / 1024.0;
  const float GEAR_RATIO            = 3.0/7.0;
  const float DEGREES_TO_RADIANS    = 2.0 * M_PI / 360.0;
  const int   WHEEL_AXLE_LEGNTH_IN  = 32;
  const int   WHEEL_BASE_LENGTH_IN  = 61;
  
  // Convert the pot diff into the pot's angle of movement
  // After, convert that to axle angle of movement based on the gear ratio
  float potAngleDiff = potDiff * POT_INPUT_RATIO;
  float axleAngleDiff = potAngleDiff * GEAR_RATIO;
  
  // From comments above:
  // r = w / tan(theta) (front wheel radius)
  // R = w / sin(theta) (back wheel radius)
  // Outer wheel radius is also the inner radius + axle length
  float innerWheelRadius = abs(static_cast<float>(WHEEL_BASE_LENGTH_IN) / tan(axleAngleDiff * DEGREES_TO_RADIANS));
  float outerWheelRadius = innerWheelRadius + static_cast<float>(WHEEL_AXLE_LEGNTH_IN);
  
  // Arclength equation is S = r * theta
  float innerArcLength = innerWheelRadius * axleAngleDiff;
  float outerArcLength = outerWheelRadius * axleAngleDiff;

  float arcLengthRatio = outerArcLength / innerArcLength;

  /*
  Serial.print("Raw pot value: ");
  Serial.println(frontAxlePotentiometerValue);
  Serial.print("Pot diff from center: ");
  Serial.println(potDiff);
  Serial.print("Pot angle diff: ");
  Serial.println(potAngleDiff);
  Serial.print("Axle angle diff: ");
  Serial.println(axleAngleDiff);
  Serial.print("Inner wheel radius: ");
  Serial.println(innerWheelRadius);
  Serial.print("Outer wheel radius: ");
  Serial.println(outerWheelRadius);
  Serial.print("Inner arc length: ");
  Serial.println(innerArcLength);
  Serial.print("Outer arc length: ");
  Serial.println(outerArcLength);
  Serial.print("Arc length ratio: ");
  Serial.println(arcLengthRatio);
  Serial.println();
  Serial.println();
  delay(2000);
  */
}

