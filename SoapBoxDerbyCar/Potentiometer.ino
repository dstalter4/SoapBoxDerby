////////////////////////////////////////////////////////////////////////////////
/// File:     Potentiometer.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for potentiometers on a soap
///           box derby car.
///
/// Edit History:
/// - dts 28-DEC-2017 Created from potentiometer methods in Sensors.ino.
/// - dts 02-JAN-2018 Silence warnings.
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
/// Method: CalibrateSteeringPotentiometer
///
/// Details:  Automatically turns the steering to max left, then to max right,
///           and finally attempts to move back to true center.
///
/// Note: Pot is wired up where left -> right is decreasing.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::CalibrateSteeringPotentiometer()
{
  static int calibrationAttempt = 0;
  Serial.println(F("Centering steering by pot..."));
  Serial.print(F("Calibration attempt #"));
  Serial.println(++calibrationAttempt);

  // @todo: Debug why this sometimes gets stuck.
  
  // Calibrate max left
  
  // Move to the left limit switch
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_LEFT_SPEED);
  
  while (m_LeftSteeringLimitSwitchValue != 1)
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
  
  while (m_RightSteeringLimitSwitchValue != 1)
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
  
  // Compute and save off the center value
  m_FrontAxlePotCenterValue = m_FrontAxlePotMaxRightValue + ((m_FrontAxlePotMaxLeftValue - m_FrontAxlePotMaxRightValue) / 2);
  
  // Return to center
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_CENTER_SPEED);
  do
  {
    // Don't read too fast, or the potentiometer values may be inaccurate
    delay(POTENTIOMETER_READ_SPACING_DELAY_MS);
    ReadPotentiometers();
  }
  while (m_FrontAxlePotentiometerValue < m_FrontAxlePotCenterValue);
  
  // Back to center, motor off
  SetSteeringSpeedControllerValue(OFF);
  
  // Pause to let things settle
  delay(AUTO_CENTERING_CALIBRATION_DELAY_MS);
  
  m_LastGoodPotValue = m_FrontAxlePotCenterValue;
  m_bCalibrationComplete = true;
  
  Serial.print(F("Left calibration pot value: "));
  Serial.println(m_FrontAxlePotMaxLeftValue);
  Serial.print(F("Right calibration pot value: "));
  Serial.println(m_FrontAxlePotMaxRightValue);
  Serial.print(F("Center calibration pot value: "));
  Serial.println(m_FrontAxlePotCenterValue);
  Serial.print(F("Final calibration position: "));
  ReadPotentiometers();
  Serial.println(m_FrontAxlePotentiometerValue);
  Serial.print(F("Pot diff: "));
  Serial.println(m_FrontAxlePotMaxLeftValue  - m_FrontAxlePotMaxRightValue);
  Serial.println(F("Centering calibration complete..."));
  Serial.println();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: CenterSteeringByPotentiometer
///
/// Details:  Automatically turns the steering back to the center value found
///           during power up calibration.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::CenterSteeringByPotentiometer()
{
  // Get the latest potetiometer value
  ReadPotentiometers();
  
  // If the pot value is greater then center, the angle is to the left,
  // so need to move back to the right.
  if (m_FrontAxlePotentiometerValue >= (m_FrontAxlePotCenterValue + POTENTIOMETER_MAX_JITTER_VALUE))
  {
    SetSteeringSpeedControllerValue(AUTO_TURN_RIGHT_SPEED);
  }
  // If the pot value is less then center, the angle is to the right,
  // so need to move back to the left.
  else if (m_FrontAxlePotentiometerValue <= (m_FrontAxlePotCenterValue - POTENTIOMETER_MAX_JITTER_VALUE))
  {
    SetSteeringSpeedControllerValue(AUTO_TURN_LEFT_SPEED);
  }
  else
  {
    SetSteeringSpeedControllerValue(OFF);
  }
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
/// 12       |  3.1743178"
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

  if (m_bCalibrationComplete)
  {
    // If the reading is too far beyond what is expected from calibration, discard
    // this current reading and use the last good one.
    if ( (m_FrontAxlePotentiometerValue > (m_FrontAxlePotMaxLeftValue + POTENTIOMETER_MAX_JITTER_VALUE)) ||
         (m_FrontAxlePotentiometerValue < (m_FrontAxlePotMaxRightValue - POTENTIOMETER_MAX_JITTER_VALUE)) )
    {
      m_FrontAxlePotentiometerValue = m_LastGoodPotValue;
    }
    // Reading was good, save it off for potential future use
    else
    {
      m_LastGoodPotValue = m_FrontAxlePotentiometerValue;
    }
  }
  
  // Current implementation only reads the value from the sensor.
  // The logic below may be useful in the future when more complex behavior is required.
  return;
  
  // Find the potentiometer's diff from true center
  int potDiff = 0;
  SteeringDirection axleDirection = NONE;
  if (m_FrontAxlePotentiometerValue > m_FrontAxlePotCenterValue)
  {
    // Axle angled to the left
    potDiff = m_FrontAxlePotentiometerValue - m_FrontAxlePotCenterValue;
    axleDirection = LEFT;
  }
  else if (m_FrontAxlePotentiometerValue < m_FrontAxlePotCenterValue)
  {
    // Axle angled to the right
    potDiff = m_FrontAxlePotCenterValue - m_FrontAxlePotentiometerValue;
    axleDirection = RIGHT;
  }
  else
  {
    potDiff = 0;
    axleDirection = NONE;
  }
  
  // The potentiometer has a 250 degree range of motion, and 1024 input value
  // GEAR_RATIO comes from the teeth ratio on the gears
  const double POT_INPUT_RATIO      = 250.0 / 1024.0;
  const double GEAR_RATIO           = 3.0/7.0;
  
  // Convert the pot diff into the pot's angle of movement
  // After, convert that to axle angle of movement based on the gear ratio
  double potAngleDiff = potDiff * POT_INPUT_RATIO;
  double axleAngleDiff = potAngleDiff * GEAR_RATIO;
  
  // From comments above:
  // r = w / tan(theta) (front wheel radius)
  // R = w / sin(theta) (back wheel radius)
  // Outer wheel radius is also the inner radius + axle length
  double innerWheelRadius = abs(WHEEL_BASE_LENGTH_INCHES / tan(axleAngleDiff * DEGREES_TO_RADIANS));
  double outerWheelRadius = innerWheelRadius + WHEEL_AXLE_LEGNTH_INCHES;
  
  // Arclength equation is S = r * theta
  double innerArcLength = innerWheelRadius * axleAngleDiff;
  double outerArcLength = outerWheelRadius * axleAngleDiff;

  double arcLengthRatio = outerArcLength / innerArcLength;
  
  // Silence warnings (for now)
  (void)axleDirection;
  (void)arcLengthRatio;

  /*
  Serial.print(F("Raw pot value: "));
  Serial.println(frontAxlePotentiometerValue);
  Serial.print(F("Pot diff from center: "));
  Serial.println(potDiff);
  Serial.print(F("Pot angle diff: "));
  Serial.println(potAngleDiff);
  Serial.print(F("Axle angle diff: "));
  Serial.println(axleAngleDiff);
  Serial.print(F("Inner wheel radius: "));
  Serial.println(innerWheelRadius);
  Serial.print(F("Outer wheel radius: "));
  Serial.println(outerWheelRadius);
  Serial.print(F("Inner arc length: "));
  Serial.println(innerArcLength);
  Serial.print(F("Outer arc length: "));
  Serial.println(outerArcLength);
  Serial.print(F("Arc length ratio: "));
  Serial.println(arcLengthRatio);
  Serial.println();
  Serial.println();
  delay(2000);
  */
}

