////////////////////////////////////////////////////////////////////////////////
/// File:     Sensors.ino
/// Author:   David Stalter
///
/// Details:  Contains the main logic and workflow for sensors on a soap box
///           derby car.
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
/// Method: ConfigureSensors
///
/// Details:  Sets up the pin mode for all sensors.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ConfigureSensors()
{
  // Configure the sensor pin modes
  pinMode(BRAKE_MAGNET_RELAY_PIN, OUTPUT);

  // @todo: There are phantom interrupts being triggered by the limit
  // switches.  Stick to polling mode until the issue can be investigated.
  // Note: Using a limit switch as an interrupt may require still configuring
  // the pin as INPUT_PULLUP or a physical resistor to be connected.
  pinMode(STEERING_LEFT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEERING_RIGHT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN, INPUT_PULLUP);
  
  // Even though the Hall sensor pins will be used for interrupts, still
  // configure them as inputs with pull-up resistors.  The pins will be read in
  // the ISR and it doesn't hurt interrupt edge detection to set them up this way.
  pinMode(LEFT_HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_HALL_SENSOR_PIN, INPUT_PULLUP);
  
  pinMode(AUTONOMOUS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SERIAL_TRANSMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SWITCH_3_RESERVED, INPUT_PULLUP);
  pinMode(SWITCH_4_RESERVED, INPUT_PULLUP);
  
  pinMode(STEERING_ENCODER_PIN, INPUT);
  pinMode(SONAR_TRIGGER_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: SteeringLimitSwitchInterruptHandler
///
/// Details:  Interrupt handler for when a steering limit switch triggers.
///           See https://playground.arduino.cc/Code/Interrupts and the AVR
///           ATmega328P datasheet.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::SteeringLimitSwitchInterruptHandler()
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
  
  volatile InterruptEdgeDirection interruptEdge = static_cast<InterruptEdgeDirection>(digitalRead(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN));
  
  // Debug visual assist (reversed logic since this is falling edge)
  int ledState = (interruptEdge == RISING_EDGE) ? LOW : HIGH;
  digitalWrite(STEER_LIMIT_SWITCHES_LED_PIN, ledState);

  if (interruptEdge == FALLING_EDGE)
  {
    // A steering limit switch is tripped, turn the motor off
    GetSingletonInstance()->DisableSteeringSpeedController();
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadLimitSwitches
///
/// Details:  Reads and stores all limit switch values.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadLimitSwitches()
{
  m_LeftSteeringLimitSwitchValue = digitalRead(STEERING_LEFT_LIMIT_SWITCH_PIN);
  m_RightSteeringLimitSwitchValue = digitalRead(STEERING_RIGHT_LIMIT_SWITCH_PIN);
  
  if ((m_LeftSteeringLimitSwitchValue == 1) || (m_RightSteeringLimitSwitchValue == 1))
  {
    digitalWrite(STEER_LIMIT_SWITCHES_LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(STEER_LIMIT_SWITCHES_LED_PIN, LOW);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadSonarSensors
///
/// Details:  Reads and stores all sonar values.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadSonarSensors()
{
  const int SONAR_TRIGGER_INIT_DELAY_US   = 2;
  const int SONAR_TRIGGER_PULSE_DELAY_US  = 5;
  const int SONAR_CONVERT_PULSE_TO_INCHES = 74 / 2;
  
  // Prepare output pin with a clean low signal
  digitalWrite(SONAR_TRIGGER_PIN, LOW);
  delayMicroseconds(SONAR_TRIGGER_INIT_DELAY_US);
  
  // Trigger the pin to start the output sonar signal
  digitalWrite(SONAR_TRIGGER_PIN, HIGH);
  delayMicroseconds(SONAR_TRIGGER_PULSE_DELAY_US);
  digitalWrite(SONAR_TRIGGER_PIN, LOW);

  // Listen for a response
  // Division constants from the sample 'Ping' program
  m_SonarDistanceInches = pulseIn(SONAR_ECHO_PIN, HIGH, PULSE_IN_TIMEOUT_US) / SONAR_CONVERT_PULSE_TO_INCHES;
}

