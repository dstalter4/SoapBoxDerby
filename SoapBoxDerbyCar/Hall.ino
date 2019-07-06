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
/// Copyright (c) 2019 David Stalter
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
  // touch interrupt enabling/disabling ourselves via
  // interrupts()/noInterrupts().
  
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
  digitalWrite(LEFT_HALL_SENSOR_LED_PIN, static_cast<int>(leftSensorInterruptEdge));
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
  // touch interrupt enabling/disabling ourselves via
  // interrupts()/noInterrupts().
  
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
  digitalWrite(RIGHT_HALL_SENSOR_LED_PIN, static_cast<int>(rightSensorInterruptEdge));
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

