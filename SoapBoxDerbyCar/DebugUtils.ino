////////////////////////////////////////////////////////////////////////////////
/// File:     DebugUtils.ino
/// Author:   David Stalter
///
/// Details:  Contains debug related functions for a soap box derby car.
///
/// Copyright (c) 2019 David Stalter
////////////////////////////////////////////////////////////////////////////////

// INCLUDES
#include "SoapBoxDerbyCar.hpp"        // for constants and function declarations

// STATIC DATA
const unsigned int SoapBoxDerbyCar::UNUSED_PINS[] =
{
  SERIAL_RX_RESERVED,         SERIAL_TX_RESERVED,
  PIN_12_RESERVED,            PIN_13_RESERVED,
  SERIAL_3_TX_RESERVED,       SERIAL_3_RX_RESERVED,
  SERIAL_2_TX_RESERVED,       SERIAL_2_RX_RESERVED,
  PIN_21_INTERRUPT_RESERVED,
  PIN_22_RESERVED,            PIN_23_RESERVED,
  PIN_24_RESERVED,            PIN_25_RESERVED,
  PIN_36_RESERVED,            PIN_37_RESERVED,
  PIN_38_RESERVED,            PIN_39_RESERVED,
  PIN_40_RESERVED,            PIN_41_RESERVED,
  PIN_42_RESERVED,            PIN_43_RESERVED,
  PIN_48_RESERVED,            PIN_49_RESERVED,
  PIN_50_RESERVED
};

// GLOBALS
// (none)


////////////////////////////////////////////////////////////////////////////////
/// Method: ConfigureDebugPins
///
/// Details:  Configures the pin mode of pins used for debug purposes.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ConfigureDebugPins()
{
  // Loop over all the debug LEDs present
  for (unsigned int i = DEBUG_OUTPUT_LEDS_START_PIN; i <= DEBUG_OUTPUT_LEDS_END_PIN; i++)
  {
    // Configure the pin as an output
    pinMode(i, OUTPUT);

    // Start with the LED off, except the initializing one which was already turned on
    if (i != INITIALIZING_LED_PIN)
    {
      digitalWrite(i, LOW);
    }
  }

  // Configure the unused pins to drive them all to known states
  for (unsigned int i = 0U; i < (sizeof(UNUSED_PINS) / sizeof(UNUSED_PINS[0])); i++)
  {
    pinMode(UNUSED_PINS[i], OUTPUT);
    digitalWrite(UNUSED_PINS[i], LOW);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ProcessAssert
///
/// Details:  Function called when an ASSERT in the code occurs.  For the soap
///           box derby car, it will cause an emergency stop and strobe the
///           debug LEDs.  This function is fatal and cannot be recovered from
///           without power cycling.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ProcessAssert()
{
  bool ledState = static_cast<bool>(HIGH);
  while (true)
  {
    if (m_pSoapBoxDerbyCar != nullptr)
    {
      EmergencyStop(m_pSoapBoxDerbyCar);
    }
    
    // Flash all the LEDs to indicate something is wrong
    for (unsigned int i = DEBUG_OUTPUT_LEDS_START_PIN; i <= DEBUG_OUTPUT_LEDS_END_PIN; i++)
    {
      digitalWrite(i, static_cast<int>(ledState));
    }

    ledState = !ledState;
    
    const int ASSERT_DELAY_MS = 1000;    
    delay(ASSERT_DELAY_MS);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: DisplayValues
///
/// Details:  Displays debug information about the inputs, sensors and other
///           state control variables.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::DisplayValues(bool bShowImmediately)
{
  static unsigned long currentTimeStamp = 0;
  static unsigned long oldTimeStamp = 0;
  static int displayCount = 0;

  // Only display things every so often to not hog the CPU
  currentTimeStamp = GetTimeStampMs();
  if (bShowImmediately || ((currentTimeStamp - oldTimeStamp) > DEBUG_PRINT_INTERVAL_MS))
  {
    Serial.print(F("Debug print #"));
    Serial.print(displayCount++);
    Serial.print(F(", "));
    Serial.print(F("Time: "));
    Serial.println(currentTimeStamp);
    
    Serial.print(F("Steering input: "));
    Serial.println(m_ControllerChannelInputs[YAW_INPUT_CHANNEL]);
    Serial.print(F("Brake input: "));
    Serial.println(m_ControllerChannelInputs[BRAKE_INPUT_CHANNEL]);
    Serial.print(F("Emergency stop input: "));
    Serial.println(m_ControllerChannelInputs[MASTER_ENABLE_INPUT_CHANNEL]);
    Serial.print(F("Steering encoder: "));
    Serial.println(m_SteeringEncoderValue);
    Serial.print(F("Steering encoder multiplier: "));
    Serial.println(m_SteeringEncoderMultiplier);
    
    Serial.print(F("Left hall count: "));
    Serial.println(m_LeftHallCount);
    Serial.print(F("Right hall count: "));
    Serial.println(m_RightHallCount);
    Serial.print(F("Left wheel distance (ft.): "));
    Serial.println(m_LeftWheelDistanceInches / INCHES_PER_FOOT);
    Serial.print(F("Right wheel distance (ft.): "));
    Serial.println(m_RightWheelDistanceInches / INCHES_PER_FOOT);
    
    Serial.print(F("Left limit switch: "));
    Serial.println(m_LeftSteeringLimitSwitchValue);
    Serial.print(F("Right limit switch: "));
    Serial.println(m_RightSteeringLimitSwitchValue);
    Serial.print(F("Brake relay state: "));
    Serial.println(m_bBrakeApplied);
    
    Serial.print(F("Front axle potentiometer: "));
    Serial.println(m_FrontAxlePotentiometerValue);
    
    Serial.print(F("Sonar sensor: "));
    Serial.println(m_SonarDistanceInches);

    Serial.print(F("Data log index: "));
    Serial.println(m_NonVolatileCarData.m_DataLogIndex);
    Serial.print(F("Data log overflowed: "));
    Serial.println(m_NonVolatileCarData.m_bDataLogOverflowed ? "true" : "false");
  
    Serial.println();
    Serial.println();

    oldTimeStamp = currentTimeStamp;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: ReadSerialInput
///
/// Details:  Gets any input from the serial console and takes action if a
///           valid command is received.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::ReadSerialInput()
{
  if (Serial.available())
  {
    switch (Serial.read())
    {
      case COMMAND_DISPLAY_DEBUG_PRINTS:
      {
        DisplayValues(true);
        break;
      }
      case COMMAND_DISPLAY_DATA_LOG:
      {
        DisplayDataLog();
        break;
      }
      case COMMAND_CLEAR_DATA_LOG:
      {
        ClearDataLog(RAM_LOG);
        break;
      }
      case COMMAND_SEND_SERIAL_DATA:
      {
        SendCarSerialData();
        break;
      }
      case COMMAND_DISPLAY_EEPROM:
      {
        DisplayEeprom();
        break;
      }
      case COMMAND_ERASE_EEPROM:
      {
        EraseEeprom();
        break;
      }
      case COMMAND_RESTORE_FROM_EEPROM:
      {
        RestoreLogFromEeprom();
        break;
      }
      case COMMAND_WRITE_TO_EEPROM:
      {
        WriteLogToEeprom();
        break;
      }
      case COMMAND_NEW_LINE:
      case COMMAND_CARRIAGE_RETURN:
      {
        break;
      }
      default:
      {
        Serial.println(F("Unrecognized input command."));
        break;
      }
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: BlinkStatusLight
///
/// Details:  Blinks an LED to indicate the program is still running.
////////////////////////////////////////////////////////////////////////////////
void SoapBoxDerbyCar::BlinkStatusLight()
{
  if (CalcDeltaTimeMs(m_StatusLedTimeStampMs) > STATUS_LED_BLINK_DELAY_MS)
  {
    digitalWrite(STATUS_LED_PIN, static_cast<int>(m_bStatusLedState));
    m_bStatusLedState = !m_bStatusLedState;
    m_StatusLedTimeStampMs = GetTimeStampMs();
  }
}

