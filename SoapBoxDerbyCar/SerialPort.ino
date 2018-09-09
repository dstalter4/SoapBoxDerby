////////////////////////////////////////////////////////////////////////////////
/// File:     SerialPort.ino
/// Author:   David Stalter
///
/// Details:  Contains functionality for serial port operations speicific to the
///           soap box derby car.  At least one of the non-default ports is used
///           to transmit data to another microcontroller.
///
/// Edit History:
/// - dts 24-JUN-2018 Created.
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
/// Method: ConfigureSerialPorts
///
/// Details:  Sets up the serial ports.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ConfigureSerialPorts()
{
  // Enable the default UART for printing
  const long int SERIAL_PORT_BAUD_RATE = 115200;
  Serial.begin(SERIAL_PORT_BAUD_RATE);
  Serial.println(F("Initializing..."));

  // Enable the serial port for transmitting car data
  m_pDataTransmitSerialPort->begin(SERIAL_PORT_BAUD_RATE);
  m_pDataTransmitSerialPort->setTimeout(SERIAL_PORT_TIMEOUT_MS);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: IsSerialTransmitSwitchSet
///
/// Details:  Checks if the switch on the car to send serial data is set.
////////////////////////////////////////////////////////////////////////////////
bool SoapBoxDerbyCar::IsSerialTransmitSwitchSet()
{
  return (digitalRead(SERIAL_TRANSMIT_SWITCH_PIN) == 1);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: IsCarDataRequested
///
/// Details:  Checks if there is a request to send serial data.
////////////////////////////////////////////////////////////////////////////////
bool SoapBoxDerbyCar::IsCarDataRequested()
{
  bool bResult = false;
  
  if (m_pDataTransmitSerialPort->available())
  {
    if (m_pDataTransmitSerialPort->readString() == SERIAL_PORT_DATA_REQUEST_STRING)
    {
      bResult = true;
    }
  }

  return bResult;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: SendCarSerialData
///
/// Details:  Sends basic information about the state of the car and its sensors
///           out via a serial port.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::SendCarSerialData()
{
  static int transmitCount = 0;
  static unsigned long lastTimeStamp = 0;
  
  const int NUM_FIELDS_TO_TRANSMIT = 10;
  int32_t serialData[NUM_FIELDS_TO_TRANSMIT] = {};
  static_assert(sizeof(serialData) == NUM_FIELDS_TO_TRANSMIT * 4, "Serial data size is not using 32 bit integers!");

  // If enough time hasn't elapsed, don't transmit
  if (CalcDeltaTimeMs(lastTimeStamp) < SERIAL_DATA_TRANSMIT_INTERVAL_MS)
  {
    return;
  }
  
  // Package together all of the data
  int transmitDataIndex = 0;
  serialData[transmitDataIndex++] = m_CurrentSteeringValue;
  serialData[transmitDataIndex++] = m_bBrakeApplied;
  serialData[transmitDataIndex++] = m_LeftHallCount;
  serialData[transmitDataIndex++] = m_RightHallCount;
  serialData[transmitDataIndex++] = m_LeftSteeringLimitSwitchValue;
  serialData[transmitDataIndex++] = m_RightSteeringLimitSwitchValue;
  serialData[transmitDataIndex++] = m_FrontAxlePotentiometerValue;
  serialData[transmitDataIndex++] = m_bIsAutonomousExecuting;

  // Make sure the buffer wasn't overrun
  ASSERT(transmitDataIndex == NUM_FIELDS_TO_TRANSMIT);

  if (DEBUG_PRINTS)
  {
    Serial.print(F("Serial data transmission #"));
    Serial.println(++transmitCount);
  }

  // The preferred way to transmit is by writing out via a byte pointer.
  // However, the listener on the other end could be something like
  // javascript, which does not provide easy direct support of manipulating
  // byte streams.  If the receiver has better support of byte manipulation
  // and conversion, the stream can be written directly instead.  Configure
  // this via the SERIAL_PORT_USE_RAW_DATA variable.

  // Send the header
  m_pDataTransmitSerialPort->println("SBDC");

  if (SERIAL_PORT_USE_RAW_DATA)
  {
    byte * pData = reinterpret_cast<byte *>(&serialData);
    for (size_t i = 0; i < sizeof(serialData); i++)
    {
      m_pDataTransmitSerialPort->write(*pData++);
    }

    // Send a newline footer to indicate the end of the data
    m_pDataTransmitSerialPort->println();
  }
  else
  {
    // Send the data
    for (size_t i = 0; i < NUM_FIELDS_TO_TRANSMIT; i++)
    {
      m_pDataTransmitSerialPort->println(serialData[i]);
    }
  }

  lastTimeStamp = GetTimeStampMs();
}

