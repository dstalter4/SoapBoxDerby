////////////////////////////////////////////////////////////////////////////////
/// File:   SoapBoxDerbyCar.hpp
/// Author: David Stalter
///
/// Details:  Contains the class variable and function declarations for a soap
///           box derby car.  These are the variables and functions intended to
///           be used in the other .ino files in this project.  It is also used
///           as the primary control of behavior via the constants.
///
/// Note:     This program is intended to be used on a Mega2560.  The Mega has
///           a clock speed of 16Mhz (62.5ns per clock cycle).  The serial
///           ports are configured at 115200 (8.6805us).  This is roughly 87us
///           per byte transmitted out a serial port.  Arduino serial port
///           buffer size is 64 bytes.
///
///           There are three forms of memory on the Mega:
///             - Flash   (256kB)
///             - SRAM    (8kB)
///             - EEPROM  (4kB)
///           Flash is where the .text section ends up as well as any strings
///           that use the F() macro.  SRAM is where the .bss/.data sections
///           end up.  EEPROM is where non-volatile user data can be stored.
///
///           When wiring things with a resitor, the resistor goes from voltage
///           to signal.  Use 1k-5k ohm resistors.  The LEDs are wired with
///           220 ohm resistors on them.
///
/// Copyright (c) 2019 David Stalter
////////////////////////////////////////////////////////////////////////////////

#ifndef SOAPBOXDERBYCAR_HPP
#define SOAPBOXDERBYCAR_HPP

// INCLUDES
#include "PwmSpeedController.hpp"     // for speed controller declarations

// MACROS
#define ASSERT(condition)           \
  do                                \
  {                                 \
    if (!(condition))               \
    {                               \
      Serial.println();             \
      Serial.println(F("ASSERT!")); \
      Serial.print(F("File: "));    \
      Serial.println(__FILE__);     \
      Serial.print(F("Line: "));    \
      Serial.println(__LINE__);     \
      ProcessAssert();              \
    }                               \
  } while (false)

#define UNUSED __attribute__((unused))
#define SECTION(x) __attribute__((section (x)))
#define STRINGIFY(s) #s
#define IGNORE_FLAGS(s) _Pragma(STRINGIFY(GCC diagnostic ignored s))


////////////////////////////////////////////////////////////////////////////////
/// Class: SoapBoxDerbyCar
///
/// Details:  Describes the functionality for a remote controlled and fully
///           autonmous soap box derby car.
////////////////////////////////////////////////////////////////////////////////
class SoapBoxDerbyCar
{
public:
  
  // Main control flow for the soap box derby car
  void Run();

  // Safety to function to perform an emergency stop
  static void EmergencyStop(SoapBoxDerbyCar * pInstance);
  
  //////////////////////////////////////////////////////////////////////////////
  /// Method: CreateSingletonInstance
  ///
  /// Details:  Creates the singleton SoapBoxDerbyCar instance.
  //////////////////////////////////////////////////////////////////////////////
  inline static void CreateSingletonInstance()
  {
    m_pSoapBoxDerbyCar = new SoapBoxDerbyCar();
  }
  
  //////////////////////////////////////////////////////////////////////////////
  /// Method: GetSingletonInstance
  ///
  /// Details:  Retrieves the singleton SoapBoxDerbyCar instance.
  //////////////////////////////////////////////////////////////////////////////
  inline static SoapBoxDerbyCar * GetSingletonInstance()
  {
    // Make sure the instance has been created
    ASSERT(m_pSoapBoxDerbyCar != nullptr);
    
    return m_pSoapBoxDerbyCar;
  }

  //////////////////////////////////////////////////////////////////////////////
  /// Method: AttachInterruptRoutines
  ///
  /// Details:  Attaches the interrupt routines for the soap box derby car.
  /// Note:     This is a separate function because some ISRs (those for the
  ///           Hall effect sensors) need the singleton to be created in order
  ///           to not lock up the board.  It must be ensured that this function
  ///           is not called until after CreateSingletonInstance().
  //////////////////////////////////////////////////////////////////////////////
  inline static void AttachInterruptRoutines()
  {
    // Make sure the instance is created
    ASSERT(m_pSoapBoxDerbyCar != nullptr);

    // Attach the ISRs
    attachInterrupt(digitalPinToInterrupt(LEFT_HALL_SENSOR_PIN), LeftHallSensorInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_SENSOR_PIN), RightHallSensorInterruptHandler, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN), SteeringLimitSwitchInterruptHandler, CHANGE);
  }

private:
  
  //////////////////////////////////////////////////////////////////////////////
  /// ENUMS
  //////////////////////////////////////////////////////////////////////////////
  
  // Steering Direction
  enum SteeringDirection
  {
    LEFT,
    RIGHT,
    CENTER,
    NONE
  };
  
  // Interrupt edge direction
  enum InterruptEdgeDirection
  {
    // These must match digital read values
    FALLING_EDGE  = LOW,
    RISING_EDGE   = HIGH
  };

  // Locations for where the data log can be kept
  enum LogLocation
  {
    RAM_LOG,
    EEPROM_LOG
  };
  
  
  //////////////////////////////////////////////////////////////////////////////
  /// STRUCTS
  //////////////////////////////////////////////////////////////////////////////
  
  // Data log entry structure
  struct DataLogEntry
  {
    uint32_t m_TimeStampMs;
    int32_t  m_LeftWheelDistanceInches;
    int32_t  m_RightWheelDistanceInches;
    int32_t  m_FrontAxlePotentiometer;
  };

  // Non-volatile data structure
  struct NonVolatileCarData
  {
    uint32_t m_Header;
    int      m_Incarnation;
    bool     m_bSavedByAuto;
    bool     m_bDataLogOverflowed;
    int      m_DataLogIndex;
  };
  
  
  //////////////////////////////////////////////////////////////////////////////
  /// MEMBER FUNCTIONS
  //////////////////////////////////////////////////////////////////////////////
  
  // CONSTRUCTOR/DESTRUCTOR
  SoapBoxDerbyCar();
  ~SoapBoxDerbyCar();

  // COPYING/ASSIGNMENT
  SoapBoxDerbyCar(const SoapBoxDerbyCar &);
  SoapBoxDerbyCar & operator=(const SoapBoxDerbyCar &);
  
  // AUTONOMOUS
  void AutonomousRoutine();
  void ExitAutonomous();
  bool IsAutonomousSwitchSet();
  void CenterSteeringByEncoder();
  void CenterSteeringByPotentiometer();
  void CenterSteeringAxle();
  
  // CONTROLLER
  void ConfigureController();
  bool IsControllerOn();
  bool IsRecalibrationRequested();
  void ReadControllerInput();
  
  // MOTOR CONTROL
  void SetSteeringDirection(int value);
  void SetSteeringSpeedControllerValue(int value);
  void UpdateSpeedControllers();

  // BRAKE CONTROL
  void ApplyBrake();
  void ArmBrake();
  void UpdateBrakeControl();
  
  // SENSORS
  void ConfigureSensors();

  // ENCODERS
  void CalibrateSteeringEncoder();
  void ReadEncoders();

  // HALL EFFECT
  static void LeftHallSensorInterruptHandler();
  static void RightHallSensorInterruptHandler();
  inline void IncrementLeftHallSensorCount() { m_LeftWheelDistanceInches = ++m_LeftHallCount * WHEEL_LENGTH_PER_MAGNET_INCHES; }
  inline void IncrementRightHallSensorCount() { m_RightWheelDistanceInches = ++m_RightHallCount * WHEEL_LENGTH_PER_MAGNET_INCHES; }
  void ResetHallSensorCounts();

  // LIMIT SWITCHES
  static void SteeringLimitSwitchInterruptHandler();
  inline void DisableSteeringSpeedController() { m_pSteeringSpeedController->SetSpeed(OFF); }
  void ReadLimitSwitches();

  // POTENTIOMETERS
  void CalibrateSteeringPotentiometer();
  void ReadPotentiometers();

  // SONAR
  void ReadSonarSensors();

  // TIMER
  static inline unsigned long GetTimeStampMs() { return millis(); }
  static inline unsigned long GetTimeStampUs() { return micros(); }
  static inline unsigned long CalcDeltaTimeMs(unsigned long startTimeMs) { return (millis() - startTimeMs); }
  static inline unsigned long CalcDeltaTimeUs(unsigned long startTimeUs) { return (micros() - startTimeUs); }

  // DATA LOGGING
  template <typename TypeToRead>
  void GenericReadFromEeprom(TypeToRead & rDataToRead, unsigned offset);
  template <typename TypeToWrite>
  void GenericWriteToEeprom(const TypeToWrite & rDataToWrite, unsigned offset);
  template <typename TypeToErase>
  void GenericEraseEeprom(const TypeToErase & rDataToErase, unsigned offset);
  
  void LogData(unsigned long entryTimeStampMs = GetTimeStampMs());
  void ClearDataLog(LogLocation logLocation);
  void DisplayDataLog();
  void DisplayEeprom();
  void RestoreLogFromEeprom();
  void WriteLogToEeprom(bool bFromAuto = false);
  void EraseEeprom();
  void GetEepromCarData(NonVolatileCarData & rCarData);

  // SERIAL PORT
  void ConfigureSerialPorts();
  bool IsSerialTransmitSwitchSet();
  bool IsCarDataRequested();
  void SendCarSerialData();
  
  // DEBUG ASSIST
  void ConfigureDebugPins();
  void BlinkStatusLight();
  void DisplayValues(bool bShowImmediately = false);
  void ReadSerialInput();
  static void ProcessAssert();
  
  
  //////////////////////////////////////////////////////////////////////////////
  /// MEMBER VARIABLES
  //////////////////////////////////////////////////////////////////////////////
  
  // AUTONOMOUS
  bool m_bIsAutonomousExecuting;
  
  // CONTROLLER
  // Channels start at '1', not '0'.  Increase array size by one for easy indexing.
  static const int NUM_CONTROLLER_INPUT_CHANNELS = 6;
  int m_ControllerChannelInputs[NUM_CONTROLLER_INPUT_CHANNELS + 1];
  bool m_bBrakeSwitch;
  bool m_bMasterEnable;
  
  // SPEED CONTROLLERS
  PwmSpeedController * m_pSteeringSpeedController;
  SteeringDirection m_SteeringDirection;
  int m_CurrentSteeringValue;

  // BRAKE CONTROL
  bool m_bBrakeApplied;
  
  // ENCODERS
  int m_SteeringEncoderValue;
  int m_SteeringEncoderMultiplier;
  
  // HALL EFFECT
  // Some are volatile because they are used in an interrupt handler.
  volatile unsigned int m_LeftHallCount;
  volatile unsigned int m_RightHallCount;
  double m_LeftWheelDistanceInches;
  double m_RightWheelDistanceInches;
  
  // LIMIT SWITCHES
  int m_LeftSteeringLimitSwitchValue;
  int m_RightSteeringLimitSwitchValue;
  
  // POTENTIOMETERS
  int m_FrontAxlePotentiometerValue;
  int m_FrontAxlePotMaxLeftValue;
  int m_FrontAxlePotMaxRightValue;
  int m_FrontAxlePotCenterValue;
  int m_LastGoodPotValue;
  
  // SONAR
  int m_SonarDistanceInches;

  // DATA LOGGING
  // 2 entries/sec for two minutes max
  // This is limited by the amount of SRAM the Arduino has (8kB).
  // It is also static so that it doesn't come off the heap and can be
  // used in the global variables post build computation by the IDE.
  // The size of DataLogEntry is currently sixteen bytes, which gives an
  // array size of 16 * 240 = 3840 bytes (47% of SRAM).  This size
  // also needs to fit in EEPROM (4kB), which it currently does, leaving
  // 256B maximum for any other non-volatile data.  The EEPROM layout is
  // the non-volatile data in the first 256B, followed by the data log.
  
  static const int            EEPROM_SIZE_BYTES                     = 4 * 1024;
  static const int            MAX_NON_VOLATILE_CAR_DATA_SIZE_BYTES  = 256;
  static const int            MAX_DATA_LOG_ENTRIES                  = 2 * 60 * 2;
  static const int            DATA_LOG_EEPROM_OFFSET                = MAX_NON_VOLATILE_CAR_DATA_SIZE_BYTES;
  static const unsigned long  DATA_LOG_ENTRY_INTERVAL_MS            = 500;
  static const bool           DATA_LOG_OVERFLOW_ALLOWED             = true;
  static const String         NON_VOLATILE_CAR_DATA_HEADER;
  
  static NonVolatileCarData m_NonVolatileCarData;
  static DataLogEntry m_DataLog[MAX_DATA_LOG_ENTRIES];
  
  static_assert(sizeof(m_NonVolatileCarData) < MAX_NON_VOLATILE_CAR_DATA_SIZE_BYTES, "Non-volatile car data too large!");
  static_assert((sizeof(m_DataLog) + sizeof(m_NonVolatileCarData)) < EEPROM_SIZE_BYTES, "Data will not fit in EEPROM!");

  // SERIAL PORTS
  static const int SERIAL_PORT_TIMEOUT_MS = 1;
  static const bool SERIAL_PORT_USE_RAW_DATA = false;
  static const String SERIAL_PORT_DATA_REQUEST_STRING;
  HardwareSerial * m_pDataTransmitSerialPort;
  
  // MISC
  bool m_bCalibrationComplete;
  bool m_bStatusLedState;
  unsigned long m_StatusLedTimeStampMs;
  
  // SINGLETON INSTANCE
  static SoapBoxDerbyCar * m_pSoapBoxDerbyCar;
  
  //////////////////////////////////////////////////////////////////////////////
  /// CONSTANTS
  /// NOTE: Try not to use multiplication for readability when setting constants.
  ///       It isn't clear what the sizes of the types are, and the implicit
  ///       conversions from the toolchain might not output what is expected.
  //////////////////////////////////////////////////////////////////////////////
  
  // AUTONOMOUS
  static const int            AUTO_CENTERING_CALIBRATION_LEFT_SPEED   = -50;
  static const int            AUTO_CENTERING_CALIBRATION_RIGHT_SPEED  =  50;
  static const int            AUTO_CENTERING_CALIBRATION_CENTER_SPEED = -50;
  static const int            AUTO_CENTERING_CALIBRATION_DELAY_MS     =  2000;
  static const int            AUTO_TURN_LEFT_SPEED                    = -80;
  static const int            AUTO_TURN_RIGHT_SPEED                   =  80;
  static const int            AUTO_HALL_SENSOR_LAUNCH_COUNT           =  3;
  static const int            AUTO_HALL_SENSOR_COUNT_MAX_DIFF         =  2;
  static const unsigned long  AUTO_MAX_LENGTH_MS                      =  300000;  // Five minutes
  
  // On the Mega, digital pins 2, 3, and 18-21 are interrupts.
  // Serial ports (Rx/Tx) at pins 0/1 (default), 19/18 (Serial1), 17/16 (Serial2), 15/14 (Serial3)
  
  // DIGITAL PINS
  static const unsigned int   SERIAL_RX_RESERVED                      = 0;
  static const unsigned int   SERIAL_TX_RESERVED                      = 1;
  static const unsigned int   CH1_INPUT_PIN                           = 2;    // Derby car yaw control
  static const unsigned int   CH2_INPUT_PIN                           = 3;
  static const unsigned int   CH3_INPUT_PIN                           = 4;
  static const unsigned int   CH4_INPUT_PIN                           = 5;    // Recalibrate derby car
  static const unsigned int   CH5_INPUT_PIN                           = 6;    // Derby car brake control
  static const unsigned int   CH6_INPUT_PIN                           = 7;    // Master enable (disable all input control)
  static const unsigned int   STEERING_SPEED_CONTROLLER_PIN           = 8;
  static const unsigned int   BRAKE_MAGNET_RELAY_PIN                  = 9;
  static const unsigned int   STEERING_LEFT_LIMIT_SWITCH_PIN          = 10;
  static const unsigned int   STEERING_RIGHT_LIMIT_SWITCH_PIN         = 11;
  static const unsigned int   PIN_12_RESERVED                         = 12;
  static const unsigned int   PIN_13_RESERVED                         = 13;
  static const unsigned int   SERIAL_3_TX_RESERVED                    = 14;
  static const unsigned int   SERIAL_3_RX_RESERVED                    = 15;
  static const unsigned int   SERIAL_2_TX_RESERVED                    = 16;
  static const unsigned int   SERIAL_2_RX_RESERVED                    = 17;
  static const unsigned int   LEFT_HALL_SENSOR_PIN                    = 18;   // Must be a board interrupt pin
  static const unsigned int   RIGHT_HALL_SENSOR_PIN                   = 19;   // Must be a board interrupt pin
  static const unsigned int   STEERING_LIMIT_SWITCHES_INTERRUPT_PIN   = 20;   // Must be a board interrupt pin
  static const unsigned int   PIN_21_INTERRUPT_RESERVED               = 21;   // Must be a board interrupt pin
  static const unsigned int   PIN_22_RESERVED                         = 22;
  static const unsigned int   PIN_23_RESERVED                         = 23;
  static const unsigned int   PIN_24_RESERVED                         = 24;
  static const unsigned int   PIN_25_RESERVED                         = 25;
  static const unsigned int   LEFT_HALL_SENSOR_LED_PIN                = 26;
  static const unsigned int   STEER_LIMIT_SWITCHES_LED_PIN            = 27;
  static const unsigned int   RIGHT_HALL_SENSOR_LED_PIN               = 28;
  static const unsigned int   BRAKE_MAGNET_RELAY_LED_PIN              = 29;
  static const unsigned int   AUTONOMOUS_READY_LED_PIN                = 30;
  static const unsigned int   STATUS_LED_PIN                          = 31;
  static const unsigned int   EEPROM_RW_LED_PIN                       = 32;
  static const unsigned int   STEERING_CALIBRATION_LED_PIN            = 33;
  static const unsigned int   INITIALIZING_LED_PIN                    = 34;
  static const unsigned int   MANUAL_CONTROL_LED_PIN                  = 34;
  static const unsigned int   AUTONOMOUS_EXECUTING_LED_PIN            = 35;
  static const unsigned int   PIN_36_RESERVED                         = 36;
  static const unsigned int   PIN_37_RESERVED                         = 37;
  static const unsigned int   PIN_38_RESERVED                         = 38;
  static const unsigned int   PIN_39_RESERVED                         = 39;
  static const unsigned int   PIN_40_RESERVED                         = 40;
  static const unsigned int   PIN_41_RESERVED                         = 41;
  static const unsigned int   PIN_42_RESERVED                         = 42;
  static const unsigned int   PIN_43_RESERVED                         = 43;
  static const unsigned int   AUTONOMOUS_SWITCH_PIN                   = 44;
  static const unsigned int   SERIAL_TRANSMIT_SWITCH_PIN              = 45;
  static const unsigned int   SWITCH_3_RESERVED                       = 46;
  static const unsigned int   SWITCH_4_RESERVED                       = 47;
  static const unsigned int   PIN_48_RESERVED                         = 48;
  static const unsigned int   PIN_49_RESERVED                         = 49;
  static const unsigned int   PIN_50_RESERVED                         = 50;
  static const unsigned int   STEERING_ENCODER_PIN                    = 51;
  static const unsigned int   SONAR_TRIGGER_PIN                       = 52;
  static const unsigned int   SONAR_ECHO_PIN                          = 53;

  static const unsigned int   DEBUG_OUTPUT_LEDS_START_PIN             = LEFT_HALL_SENSOR_LED_PIN;
  static const unsigned int   DEBUG_OUTPUT_LEDS_END_PIN               = AUTONOMOUS_EXECUTING_LED_PIN;
  static const unsigned int   UNUSED_PINS[];
  
  // ANALOG PINS
  static const unsigned int   FRONT_AXLE_POTENTIOMETER_PIN            = 0;
  
  // MOTOR CONTROL
  static const int            MIN_OUTPUT_PERCENTAGE                   =  10;
  static const int            RELEASE_BRAKE_PERCENTAGE                =  25;
  static const int            APPLY_BRAKE_PERCENTAGE                  = -40;
  
  // I/O
  static const int            YAW_INPUT_CHANNEL                       = 1;
  static const int            RECALIBRATE_INPUT_CHANNEL               = 4;
  static const int            BRAKE_INPUT_CHANNEL                     = 5;
  static const int            MASTER_ENABLE_INPUT_CHANNEL             = 6;
  static const int            NUM_MAGNETS_PER_WHEEL                   = 12;
  static const int            POTENTIOMETER_MAX_JITTER_VALUE          = 5;
  static const int            POTENTIOMETER_MAX_VALUE                 = 1024;
  static const int            ENCODER_MAX_VALUE                       = 4096;

  // PHYSICAL CAR CONSTANTS
  static constexpr double     WHEEL_AXLE_LEGNTH_INCHES                = 32.0;
  static constexpr double     WHEEL_BASE_LENGTH_INCHES                = 61.0;
  static constexpr double     WHEEL_DIAMETER_INCHES                   = 12.125;
  static constexpr double     WHEEL_CIRCUMFERENCE_INCHES              = M_PI * WHEEL_DIAMETER_INCHES;
  static constexpr double     WHEEL_LENGTH_PER_MAGNET_INCHES          = WHEEL_CIRCUMFERENCE_INCHES / NUM_MAGNETS_PER_WHEEL;
  
  // MISC
  static const int            OFF                                     = 0;
  static const int            ON                                      = 100;
  static const unsigned int   TENTH_OF_A_SECOND_DELAY_MS              = 100;
  static const unsigned long  STATUS_LED_BLINK_DELAY_MS               = 500;
  static const unsigned long  SERIAL_DATA_TRANSMIT_INTERVAL_MS        = 1000;
  static const unsigned long  PULSE_IN_TIMEOUT_US                     = 50000;
  static constexpr double     INCHES_PER_FOOT                         = 12.0;
  static constexpr double     DEGREES_TO_RADIANS                      = 2.0 * M_PI / 360.0;
  
  // DEBUG ASSIST
  static const char           COMMAND_DISPLAY_DEBUG_PRINTS            = 'p';
  static const char           COMMAND_DISPLAY_DATA_LOG                = 'l';
  static const char           COMMAND_CLEAR_DATA_LOG                  = 'c';
  static const char           COMMAND_SEND_SERIAL_DATA                = 's';
  static const char           COMMAND_DISPLAY_EEPROM                  = 'd';
  static const char           COMMAND_ERASE_EEPROM                    = 'e';
  static const char           COMMAND_RESTORE_FROM_EEPROM             = 'r';
  static const char           COMMAND_WRITE_TO_EEPROM                 = 'w';
  static const char           COMMAND_NEW_LINE                        = '\n';
  static const char           COMMAND_CARRIAGE_RETURN                 = '\r';
  static const bool           DEBUG_PRINTS                            = false;
  static const bool           DEBUG_COMMANDS                          = true;
  static const unsigned long  DEBUG_PRINT_INTERVAL_MS                 = 3000;
};

// The car object comes off the heap (not included in memory usage analysis).
// Make sure the size is reasonable to prevent strange runtime issues.
static_assert(sizeof(SoapBoxDerbyCar) < 256, "Instance size greater than 256B, review memory use!");

#endif // SOAPBOXDERBYCAR_HPP

