////////////////////////////////////////////////////////////////////////////////
/// File:     Encoder.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for encoders on a soap box
///           derby car.
///
/// Edit History:
/// - dts 28-DEC-2017 Created from encoder methods in Sensors.ino.
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
/// Method: CenterSteeringByEncoder
///
/// Details:  Automatically turns the steering to max left, then to max right,
///           and finally attempts to move back to true center.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::CenterSteeringByEncoder()
{
  // @todo: This should be part of calibrate, not center.
  
  Serial.println(F("Centering steering by encoder..."));
  
  // Calibrate max left
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_LEFT_SPEED);
  while (m_LeftSteeringLimitSwitchValue != 1)
  {
    ReadLimitSwitches();
    ReadEncoders();
  }
  SetSteeringSpeedControllerValue(OFF);
  int leftEncoderCalibrationValue = m_SteeringEncoderValue;
  int leftEncoderRange = (ENCODER_MAX_VALUE * m_SteeringEncoderMultiplier) + leftEncoderCalibrationValue;

  // Calibrate max right
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_RIGHT_SPEED);
  while (m_RightSteeringLimitSwitchValue != 1)
  {
    ReadLimitSwitches();
    ReadEncoders();   
  }
  SetSteeringSpeedControllerValue(OFF);
  int rightEncoderCalibrationValue = m_SteeringEncoderValue;
  int rightEncoderRange = (ENCODER_MAX_VALUE * m_SteeringEncoderMultiplier) + rightEncoderCalibrationValue;

  // @todo: Check and update based on the multiplier to calculate max left/right.
  
  // Attempt to find and move to center
  int totalEncoderRange = rightEncoderRange - leftEncoderRange;
  int centerEncoderPosition = rightEncoderRange - (totalEncoderRange / 2);
  SetSteeringSpeedControllerValue(AUTO_CENTERING_CALIBRATION_LEFT_SPEED);
  while (((m_SteeringEncoderValue + (ENCODER_MAX_VALUE * m_SteeringEncoderMultiplier)) > centerEncoderPosition) && (m_LeftSteeringLimitSwitchValue != 1))
  {
    ReadLimitSwitches();
    ReadEncoders();
  }
  SetSteeringSpeedControllerValue(OFF);

  // @todo: If the starting position is very
  // far right, the center value ends up
  // being negative and we fail to calibrate.
  Serial.print(F("Current value: "));
  Serial.println(m_SteeringEncoderValue);
  Serial.print(F("Multiplier: "));
  Serial.println(m_SteeringEncoderMultiplier);
  Serial.print(F("Left encoder calibration: "));
  Serial.println(leftEncoderCalibrationValue);
  Serial.print(F("Right encoder calibration: "));
  Serial.println(rightEncoderCalibrationValue);
  Serial.print(F("Left encoder range: "));
  Serial.println(leftEncoderRange);
  Serial.print(F("Right encoder range: "));
  Serial.println(rightEncoderRange);
  Serial.print(F("Total range: "));
  Serial.println(totalEncoderRange);
  Serial.print(F("Center encoder position: "));
  Serial.println(centerEncoderPosition);
  Serial.println();

  // If the left limit switch tripped again, we
  // went all the way back to the left and failed
  // to properly find the center.  If the right
  // limit switch tripped, we never tried to go
  // back to the center.
  if (m_LeftSteeringLimitSwitchValue == 1 || m_RightSteeringLimitSwitchValue == 1)
  {
    Serial.println(F("CALIBRATION FAILED!!!"));
  }
  else
  {
    Serial.println(F("CALIBRATION SUCCESSFUL"));
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: CalibrateSteeringEncoder
///
/// Details:  Gathers information about the encoder and its readings.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::CalibrateSteeringEncoder()
{
  // @todo: Calibrate under full load, otherwise results are meaningless.
  // @todo: Iterate left/right, relying on the limit switches
  // Use caution with this loop - it does not look at the limit switches.
  // Right now it should only be called with no load.
  for (uint32_t i = 0U; i < 128; i++)
  {
    // First time through discard the data
    static bool bFirst = true;

    // Get and measure the data
    uint32_t startSteeringEncoderValue = pulseIn(STEERING_ENCODER_PIN, HIGH, PULSE_IN_TIMEOUT_US);
    uint32_t startTime = GetTimeStampMs();
    SetSteeringSpeedControllerValue(100);
    delay(100);
    SetSteeringSpeedControllerValue(OFF);
    uint32_t endTime = GetTimeStampMs();
    uint32_t endSteeringEncoderValue = pulseIn(STEERING_ENCODER_PIN, HIGH, PULSE_IN_TIMEOUT_US);
    uint32_t deltaSteeringEncoderValue = 0U;
    
    if (bFirst)
    {
      Serial.println(F("First discarded."));
      Serial.println();
      bFirst = false;
      delay(5000);
    }
    else
    {
      // Positive control causes the encoder to count down
      // start > end is the normal case
      if (startSteeringEncoderValue > endSteeringEncoderValue)
      {
        deltaSteeringEncoderValue = startSteeringEncoderValue - endSteeringEncoderValue;
      }
      // end > start, rollover case
      else
      {
        deltaSteeringEncoderValue = (4096 - endSteeringEncoderValue) + startSteeringEncoderValue;
      }
  
      static uint32_t deltaSteeringEncoderLow = deltaSteeringEncoderValue;
      static uint32_t deltaSteeringEncoderHigh = deltaSteeringEncoderValue;
      
      // Save off high/low
      if (deltaSteeringEncoderValue < deltaSteeringEncoderLow)
      {
        deltaSteeringEncoderLow = deltaSteeringEncoderValue;
      }
      if (deltaSteeringEncoderValue > deltaSteeringEncoderHigh)
      {
        deltaSteeringEncoderHigh = deltaSteeringEncoderValue;
      }
  
      static uint64_t totalSteeringEncoder = 0ULL;
      totalSteeringEncoder += deltaSteeringEncoderValue;
      static uint32_t count = 0U;
      count++;
      
      Serial.print(F("Encoder Start: "));
      Serial.println(startSteeringEncoderValue);
      Serial.print(F("Encoder End: "));
      Serial.println(endSteeringEncoderValue);
      Serial.print(F("Encoder Delta: "));
      Serial.println(deltaSteeringEncoderValue);
      Serial.print(F("Encoder Low: "));
      Serial.println(deltaSteeringEncoderLow);
      Serial.print(F("Encoder High: "));
      Serial.println(deltaSteeringEncoderHigh);
      Serial.print(F("Encoder Average: "));
      Serial.println(static_cast<int>(totalSteeringEncoder / count));
      
      //Serial.print(F("Start Time: "));
      //Serial.println(startTime);
      //Serial.print(F("End Time: "));
      //Serial.println(endTime);
      Serial.print(F("Time Delta: "));
      Serial.println(endTime - startTime);
      
      Serial.println();
      Serial.println();
      
      delay(100);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadEncoders
///
/// Details:  Reads and stores all encoder values.  This method also contains
///           some logic to handle encoder overflow by monitoring encoder
///           direction of change and using a multiplier since the encoder
///           is relative with a range of 0 - 4096.
///
/// Note: http://www.ctr-electronics.com/downloads/pdf/Magnetic%20Encoder%20User's%20Guide.pdf
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadEncoders()
{
  // This method should not currently be called
  ASSERT(false);
  
  static bool bFirstTime = true;
  static int previousSteeringEncoderValue = -1;

  // The first time through we need to get a previous value
  // so that any subsequent calls have valid data to use.
  if (bFirstTime)
  {
    previousSteeringEncoderValue = pulseIn(STEERING_ENCODER_PIN, HIGH, PULSE_IN_TIMEOUT_US);
    m_SteeringEncoderMultiplier = 0;
    bFirstTime = false;
  }
  else
  {
    // Save previous value, no further action needed
    previousSteeringEncoderValue = m_SteeringEncoderValue;
  }

  // Get a new sample reading for this call
  m_SteeringEncoderValue = pulseIn(STEERING_ENCODER_PIN, HIGH, PULSE_IN_TIMEOUT_US);

  // @todo: Have this use the multiplier!
  // Save off minimum value seen
  /*
  if (steeringEncoderValue < minSteeringEncoderValue)
  {
    minSteeringEncoderValue = steeringEncoderValue;
  }

  // Save off maximum value seen
  if (steeringEncoderValue > maxSteeringEncoderValue)
  {
    maxSteeringEncoderValue = steeringEncoderValue;
  }
  */
  
  // This section of code checks for overflow/underflow of the encoder.
  // The data read on the pulseIn has empirically shown to not be constant,
  // even if the steering mechanism is not moving.  To ensure that we are
  // seeing a 'new' reading instead of variations from the same physical
  // position, make sure the difference of the two numbers exceeds the
  // steering jitter.  This logic *might* handle the case where the
  // absolute position is near zero by ensuring too large of a delta is
  // also not seen.
  const int STEERING_ENCODER_JITTER_MIN = 1000;
  const int STEERING_ENCODER_JITTER_MAX = 4050;

  int steeringEncoderDifference = abs(m_SteeringEncoderValue - previousSteeringEncoderValue);
  if ((steeringEncoderDifference > STEERING_ENCODER_JITTER_MIN) && (steeringEncoderDifference < STEERING_ENCODER_JITTER_MAX))
  {
    // If the speed controller indicates we are moving left
    if (m_SteeringDirection == LEFT)
    {
      // If we are going left (counting down) and see a jump
      // back up, we rolled under.
      if (previousSteeringEncoderValue < m_SteeringEncoderValue)
      {
        m_SteeringEncoderMultiplier--;
      }
    }
    // If the speed controller indicates we are moving right
    else if (m_SteeringDirection == RIGHT)
    {
      // If we are going right (counting up) and see a jump
      // back down, we rolled over.
      if (previousSteeringEncoderValue > m_SteeringEncoderValue)
      {
        m_SteeringEncoderMultiplier++;
      }
    }
    else
    {
    }
  }
}

