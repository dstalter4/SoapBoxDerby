////////////////////////////////////////////////////////////////////////////////
/// File:     Hall.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for Hall sensors on a soap
///           box derby car.  See https://playground.arduino.cc/Code/Interrupts
///           for information on using Arduino interrupts.  Also see
///           https://www.sunfounder.com/learn/sensor_kit_v1_for_Arduino/lesson-1-hall-sensor-sensor-kit-v1-for-arduino.html
///           for some guidance on the Hall sensor.
///
/// Edit History:
/// - dts 28-DEC-2017 Created from Hall sensor methods in Sensors.ino.
/// - dts 02-JAN-2018 Change interrupt handlers to not touch interrupt enable
///                   state and increment counters only on rising edges.
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
/// Method: LeftHallSensorInterruptHandler
///
/// Details:  Interrupt handler for when the left hall sensor triggers.
///           See https://playground.arduino.cc/Code/Interrupts and the AVR
///           ATmega328P datasheet.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::LeftHallSensorInterruptHandler()
{
  // The Atmel architecture guarantees that the Global
  // Interrupt Enable bit (SREG.I) is cleared on interrupt
  // entry.  A RETI instruction on ISR exit will re-enable
  // this bit.  Executing interrupts() and noInterrupts()
  // expand to SEI and CLI instructions, which could turn
  // nested interrupts back on.  This implies the low level
  // operating system code does not keep track of nested
  // interrupt levels.  We'll assume that the context
  // switching code properly handles all of this and not
  // touch interrupt enabling/disable ourselves.
  
  // 'volatile' because this is an ISR and the Arduino
  // documentation recommends it.
  static volatile InterruptEdgeDirection leftSensorInterruptEdge;
  
  // Get the value of the pin so we can distinguish
  // if this is a rising or falling edge interrupt.
  leftSensorInterruptEdge = static_cast<InterruptEdgeDirection>(!digitalRead(LEFT_HALL_SENSOR_PIN));
  
  // Only increment the sensor value if we see a
  // rising edge (magnetic field appearing).
  if (leftSensorInterruptEdge == RISING_EDGE)
  {
    // Increase the counter
    GetSingletonInstance()->IncrementLeftHallSensorCount();
  }
  
  // Update the visual LED
  digitalWrite(DEBUG_OUTPUT_1_LED_PIN, static_cast<int>(leftSensorInterruptEdge));
}


////////////////////////////////////////////////////////////////////////////////
/// Method: RightHallSensorInterruptHandler
///
/// Details:  Interrupt handler for when the right hall sensor triggers.
///           See https://playground.arduino.cc/Code/Interrupts and the AVR
///           ATmega328P datasheet.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::RightHallSensorInterruptHandler()
{
  // The Atmel architecture guarantees that the Global
  // Interrupt Enable bit (SREG.I) is cleared on interrupt
  // entry.  A RETI instruction on ISR exit will re-enable
  // this bit.  Executing interrupts() and noInterrupts()
  // expand to SEI and CLI instructions, which could turn
  // nested interrupts back on.  This implies the low level
  // operating system code does not keep track of nested
  // interrupt levels.  We'll assume that the context
  // switching code properly handles all of this and not
  // touch interrupt enabling/disable ourselves.
  
  // 'volatile' because this is an ISR and the Arduino
  // documentation recommends it.
  static volatile InterruptEdgeDirection rightSensorInterruptEdge;
  
  // Get the value of the pin so we can distinguish
  // if this is a rising or falling edge interrupt.
  rightSensorInterruptEdge = static_cast<InterruptEdgeDirection>(!digitalRead(RIGHT_HALL_SENSOR_PIN));
  
  // Only increment the sensor value if we see a
  // rising edge (magnetic field appearing).
  if (rightSensorInterruptEdge == RISING_EDGE)
  {
    // Increase the counter
    GetSingletonInstance()->IncrementRightHallSensorCount();
  }
  
  // Update the visual LED
  digitalWrite(DEBUG_OUTPUT_2_LED_PIN, static_cast<int>(rightSensorInterruptEdge));
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ResetHallSensorCounts
///
/// Details:  Resets the values of the hall sensor counters to zero.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ResetHallSensorCounts()
{
  m_LeftHallCount = 0;
  m_RightHallCount = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadHallSensors
///
/// Details:  Reads the state of the hall sensors.  This method also keeps
///           track of state changes and as such counts accoridngly.
///
/// Note: This function should only be used in polling implementations and not
///       simultaneously with the interrupt based approach.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadHallSensors()
{
  // Current implementation is interrupt based.
  // This function should never be called.
  ASSERT(false);
  
  // Read the Hall sensors (no magnetic field detected reads as zero)
  m_LeftHallSensorValue = !digitalRead(LEFT_HALL_SENSOR_PIN);
  m_RightHallSensorValue = !digitalRead(RIGHT_HALL_SENSOR_PIN);

  // Keep track of whether or not the sensor has been seen
  static bool bSawLeft = false;
  if (m_LeftHallSensorValue != 0)
  {
    // Only record this reading if we haven't already
    // seen it.  The sensor readings could come in so
    // fast that we sample a '1' more than once before
    // the magnet fully passes.
    if (!bSawLeft)
    {
      bSawLeft = true;
      m_LeftHallCount++;
      digitalWrite(DEBUG_OUTPUT_1_LED_PIN, HIGH);
    }
  }
  else
  {
    // The magnetic field either isn't present or has
    // fully passed.  Indicate we can see another one.
    bSawLeft = false;
    digitalWrite(DEBUG_OUTPUT_1_LED_PIN, LOW);
  }

  // Keep track of whether or not the sensor has been seen
  static bool bSawRight = false;
  if (m_RightHallSensorValue != 0)
  {
    // Only record this reading if we haven't already
    // seen it.  The sensor readings could come in so
    // fast that we sample a '1' more than once before
    // the magnet fully passes.
    if (!bSawRight)
    {
      bSawRight = true;
      m_RightHallCount++;
      digitalWrite(DEBUG_OUTPUT_2_LED_PIN, HIGH);
    }
  }
  else
  {
    // The magnetic field either isn't present or has
    // fully passed.  Indicate we can see another one.
    bSawRight = false;
    digitalWrite(DEBUG_OUTPUT_2_LED_PIN, LOW);
  }
}

