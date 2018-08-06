////////////////////////////////////////////////////////////////////////////////
/// File:     DataLogging.ino
/// Author:   David Stalter
///
/// Details:  Contains functionality for serial port operations speicific to the
///           soap box derby car.  At least one of the non-default ports is used
///           to transmit data to another microcontroller.
///
/// Edit History:
/// - dts 05-JUL-2018 Created.
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
IGNORE_FLAGS("-Wignored-qualifiers")  // Silence warnings in EEPROM.h header
#include <EEPROM.h>                   // for erasing/writing EEPROM
#include "SoapBoxDerbyCar.hpp"        // for constants and function declarations

// STATIC DATA
// (none)

// GLOBALS
// (none)


////////////////////////////////////////////////////////////////////////////////
/// Method: GenericReadFromEeprom
///
/// Details:  Reads an arbitrary data type from EEPROM at the specified offset.
////////////////////////////////////////////////////////////////////////////////
template <typename TypeToRead>
void SoapBoxDerbyCar::GenericReadFromEeprom(TypeToRead & rDataToRead, unsigned offset)
{
  digitalWrite(EEPROM_RW_LED_PIN, HIGH);
  
  byte * pData = reinterpret_cast<byte *>(&rDataToRead);
  for (size_t i = 0; i < sizeof(TypeToRead); i++)
  {
    *pData++ = EEPROM.read(offset + i);
  }

  digitalWrite(EEPROM_RW_LED_PIN, LOW);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: GenericWriteToEeprom
///
/// Details:  Writes an arbitrary data type to EEPROM at the specified offset.
////////////////////////////////////////////////////////////////////////////////
template <typename TypeToWrite>
void SoapBoxDerbyCar::GenericWriteToEeprom(const TypeToWrite & rDataToWrite, unsigned offset)
{
  digitalWrite(EEPROM_RW_LED_PIN, HIGH);
  
  const byte * pData = reinterpret_cast<const byte *>(&rDataToWrite);
  for (size_t i = 0; i < sizeof(TypeToWrite); i++)
  {
    // To save on cycles of the EEPROM, only write data that's different
    if (EEPROM.read(offset + i) != *pData)
    {
      EEPROM.write(offset + i, *pData);
    }

    pData++;
  }

  digitalWrite(EEPROM_RW_LED_PIN, LOW);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: GenericEraseEeprom
///
/// Details:  Erases an arbitrary amount of EEPROM at the specified offset.
////////////////////////////////////////////////////////////////////////////////
template <typename TypeToErase>
void SoapBoxDerbyCar::GenericEraseEeprom(const TypeToErase & rDataToErase, unsigned offset)
{
  digitalWrite(EEPROM_RW_LED_PIN, HIGH);

  // Since this template method uses the size of the type
  // that instantiated it to determine how much to erase,
  // erasing all of EEPROM can be tricky (since it would
  // require a type that is the same size as EEPROM itself,
  // and that would take up too much of RAM to implement).
  // As such, a little work around is put in place here.
  // If the argument is a reference to the nullptr, the
  // size will be all of EEPROM.  Otherwise use the size
  // of the passed in type.
  size_t sizeToErase = (&rDataToErase == nullptr) ? EEPROM.length() : sizeof(TypeToErase);
  
  for (size_t i = 0; i < sizeToErase; i++)
  {
      // To save on cycles of the EEPROM, only write data that's different
      if (EEPROM.read(offset + i) != 0xFF)
      {
        EEPROM.write(offset + i, 0xFF);
      }
  }

  digitalWrite(EEPROM_RW_LED_PIN, LOW);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: LogData
///
/// Details:  Adds an entry to the data log.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::LogData(unsigned long entryTimeStampMs)
{
  static unsigned long lastTimeStamp = 0;
  unsigned long currentTimeStamp = GetTimeStampMs();

  // If enough time hasn't elapsed, don't add another entry
  if (CalcDeltaTimeMs(lastTimeStamp) < DATA_LOG_ENTRY_INTERVAL_MS)
  {
    return;
  }
  
  if (!m_NonVolatileCarData.m_bDataLogOverflowed || DATA_LOG_OVERFLOW_ALLOWED)
  {
    m_DataLog[m_NonVolatileCarData.m_DataLogIndex].m_TimeStampMs = entryTimeStampMs;
    m_DataLog[m_NonVolatileCarData.m_DataLogIndex].m_LeftWheelDistanceInches = m_LeftWheelDistanceInches;
    m_DataLog[m_NonVolatileCarData.m_DataLogIndex].m_RightWheelDistanceInches = m_RightWheelDistanceInches;
    m_DataLog[m_NonVolatileCarData.m_DataLogIndex].m_FrontAxlePotentiometer = m_FrontAxlePotentiometerValue;
  }
  else
  {
    return;
  }

  // Check for the log to have overflowed
  m_NonVolatileCarData.m_DataLogIndex++;
  if (m_NonVolatileCarData.m_DataLogIndex == MAX_DATA_LOG_ENTRIES)
  {
    m_NonVolatileCarData.m_DataLogIndex = 0;
    m_NonVolatileCarData.m_bDataLogOverflowed = true;
  }

  lastTimeStamp = currentTimeStamp;
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ClearDataLog
///
/// Details:  Clears the full data log from EEPROM.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ClearDataLog(LogLocation logLocation)
{
  if (logLocation == RAM_LOG)
  {
    memset(&m_DataLog, 0, sizeof(m_DataLog));
  }
  else if (logLocation == EEPROM_LOG)
  {
    // Data log is offset in EEPROM
    GenericEraseEeprom(m_DataLog, DATA_LOG_EEPROM_OFFSET);
  }
  else
  {
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: DisplayDataLog
///
/// Details:  Displays the full data log out the serial port.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::DisplayDataLog()
{
  for (int i = 0; i < MAX_DATA_LOG_ENTRIES; i++)
  {
    Serial.print(F("Entry #"));
    Serial.print(i);
    Serial.print(F(" - Timestamp (ms): "));
    Serial.print(m_DataLog[i].m_TimeStampMs);
    Serial.print(F(", Left Wheel Distance (in.): "));
    Serial.print(m_DataLog[i].m_LeftWheelDistanceInches);
    Serial.print(F(", Right Wheel Distance (in.): "));
    Serial.print(m_DataLog[i].m_RightWheelDistanceInches);
    Serial.print(F(", Front Axle Potentiometer: "));
    Serial.println(m_DataLog[i].m_FrontAxlePotentiometer);
  }

  Serial.println();
  Serial.println();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: DisplayEeprom
///
/// Details:  Displays the full contents of EEPROM out the serial port.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::DisplayEeprom()
{
  digitalWrite(EEPROM_RW_LED_PIN, HIGH);

  Serial.println(F("EEPROM"));
  Serial.println(F("------"));
  
  for (size_t i = 0; i < EEPROM.length(); i++)
  {
    const size_t DISPLAY_BYTES_PER_LINE = 16;
    if ((i % DISPLAY_BYTES_PER_LINE) == 0)
    {
      Serial.println();
      Serial.print(F("0x"));
      Serial.print(i, HEX);
      Serial.print(F(": "));
    }

    Serial.print(EEPROM.read(i), HEX);
    Serial.print(F(" "));
  }

  Serial.println();
  Serial.println();
  
  digitalWrite(EEPROM_RW_LED_PIN, LOW);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: RestoreLogFromEeprom
///
/// Details:  Restores the full data log from non-volatile EEPROM.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::RestoreLogFromEeprom()
{
  GenericReadFromEeprom(m_DataLog, DATA_LOG_EEPROM_OFFSET);
  GenericReadFromEeprom(m_NonVolatileCarData.m_bDataLogOverflowed, offsetof(NonVolatileCarData, m_bDataLogOverflowed));
  GenericReadFromEeprom(m_NonVolatileCarData.m_DataLogIndex, offsetof(NonVolatileCarData, m_DataLogIndex));
}


////////////////////////////////////////////////////////////////////////////////
/// Method: WriteLogToEeprom
///
/// Details:  Writes the full data log to non-volatile EEPROM.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::WriteLogToEeprom(bool bFromAuto)
{
  GenericWriteToEeprom(m_DataLog, DATA_LOG_EEPROM_OFFSET);

  NonVolatileCarData eepromCarData;
  GetEepromCarData(eepromCarData);

  // This is a little bit of a hack.  The header size in the struct declaration
  // is 32-bits.  The header is filled with four characters, which is the same
  // size.  This is exploited to easily read/write the header, even though the
  // types are fundamentally different (uint32_t vs. String).
  for (size_t i = 0; i < sizeof(eepromCarData.m_Header); i++)
  {
    GenericWriteToEeprom(NON_VOLATILE_CAR_DATA_HEADER[i], offsetof(NonVolatileCarData, m_Header) + i);
  }
  
  GenericWriteToEeprom(m_NonVolatileCarData.m_Incarnation, offsetof(NonVolatileCarData, m_Incarnation));
  
  m_NonVolatileCarData.m_bSavedByAuto = bFromAuto;
  GenericWriteToEeprom(bFromAuto, offsetof(NonVolatileCarData, m_bSavedByAuto));
  
  GenericWriteToEeprom(m_NonVolatileCarData.m_bDataLogOverflowed, offsetof(NonVolatileCarData, m_bDataLogOverflowed));
  GenericWriteToEeprom(m_NonVolatileCarData.m_DataLogIndex, offsetof(NonVolatileCarData, m_DataLogIndex));
}


////////////////////////////////////////////////////////////////////////////////
/// Method: EraseEeprom
///
/// Details:  Erases all of EEPROM.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::EraseEeprom()
{
  nullptr_t & r = *static_cast<nullptr_t *>(nullptr);
  GenericEraseEeprom(r, 0x0);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: GetEepromCarData
///
/// Details:  Gets the non-volatile car data section from EEPROM.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::GetEepromCarData(NonVolatileCarData & rCarData)
{
  GenericReadFromEeprom(rCarData, 0x0);
}

