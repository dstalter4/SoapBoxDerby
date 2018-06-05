////////////////////////////////////////////////////////////////////////////////
/// File:   SoapBoxDerbyCar.hpp
/// Author: David Stalter
///
/// Details:  Contains the class variable and function declarations for a soap
///           box derby car.  These are the variables and functions intended to
///           be used in the other .ino files in this project.  It is also used
///           as the primary control of behavior via the constants.
///
/// Note:     When wiring things with a resitor, the resistor goes from voltage
///           to signal.  Use 1k-5k ohm resistors.
///
/// Edit History:
/// - dts 19-OCT-2017 Documentation and headers added.
/// - dts 16-DEC-2017 Add second limit switch for brake and potentiometer.
/// - dts 28-DEC-2017 Switched to SoapBoxDerbyCar class based approach, update
///                   copyright and add GPL header.
/// - dts 02-JAN-2018 Add InterruptEdgeDirection enum.  Silence warnings.
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

#ifndef SOAPBOXDERBYCAR_HPP
#define SOAPBOXDERBYCAR_HPP

// INCLUDES
#include "PwmSpeedController.hpp"     // for speed controller declarations

// MACROS
#define ASSERT(condition)         \
  do                              \
  {                               \
    if (!(condition))             \
    {                             \
      Serial.println();           \
      Serial.println("ASSERT!");  \
      Serial.print("File: ");     \
      Serial.println(__FILE__);   \
      Serial.print("Line: ");     \
      Serial.println(__LINE__);   \
      ProcessAssert();            \
    }                             \
  } while (false)


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
    // @todo: Figure out why this is occasionally tripping.
    // Make sure the instance has been created
    //ASSERT(m_pSoapBoxDerbyCar != nullptr);
    
    return m_pSoapBoxDerbyCar;
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
  void ApplyBrake(bool bWaitForDone = false);
  void ReleaseBrake(bool bWaitForDone = false);
  void UpdateSpeedControllers();
  
  // SENSORS
  void ConfigureSensors();
  
  void CalibrateSteeringEncoder();
  void ReadEncoders();
  
  static void LeftHallSensorInterruptHandler();
  static void RightHallSensorInterruptHandler();
  inline void IncrementLeftHallSensorCount() { m_LeftHallCount++; }
  inline void IncrementRightHallSensorCount() { m_RightHallCount++; }
  void ResetHallSensorCounts();
  void ReadHallSensors();

  static void SteeringLimitSwitchInterruptHandler();
  static void BrakeLimitSwitchInterruptHandler();
  inline void DisableSteeringSpeedController() { m_pSteeringSpeedController->SetSpeed(OFF); }
  inline void DisableBrakeSpeedController() { m_pBrakeSpeedController->SetSpeed(OFF); }
  void ReadLimitSwitches();
  
  void CalibrateSteeringPotentiometer();
  void ReadPotentiometers();
  
  void ReadSonarSensors();
  
  // DEBUG ASSIST
  void ConfigureDebugPins();
  void DisplayValues();
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
  PwmSpeedController * m_pBrakeSpeedController;
  bool m_bBrakeApplied;
  
  // ENCODERS
  int m_SteeringEncoderValue;
  int m_SteeringEncoderMultiplier;
  
  // HALL SENSORS
  // Some are volatile because they are used in an interrupt handler.
  int m_LeftHallSensorValue;
  int m_RightHallSensorValue;
  volatile unsigned int m_LeftHallCount;
  volatile unsigned int m_RightHallCount;
  
  // LIMIT SWITCHES
  int m_LeftSteeringLimitSwitchValue;
  int m_RightSteeringLimitSwitchValue;
  int m_BrakeReleaseLimitSwitchValue;
  int m_BrakeApplyLimitSwitchValue;
  
  // POTENTIOMETERS
  int m_FrontAxlePotentiometerValue;
  int m_FrontAxlePotMaxLeftValue;
  int m_FrontAxlePotMaxRightValue;
  int m_FrontAxlePotCenterValue;
  int m_LastGoodPotValue;
  
  // SONAR SENSORS
  int m_SonarDistanceInches;
  
  // MISC
  SteeringDirection m_SteeringDirection;
  bool m_bCalibrationComplete;
  
  // SINGLETON INSTANCE
  static SoapBoxDerbyCar * m_pSoapBoxDerbyCar;
  
  //////////////////////////////////////////////////////////////////////////////
  /// CONSTANTS
  /// NOTE: Try not to use multiplication for readability when setting constants.
  ///       It isn't clear what the sizes of the types are, and the implicit
  ///       conversions from the toolchain might not output what is expected.
  //////////////////////////////////////////////////////////////////////////////
  
  // AUTONOMOUS
  static const int            AUTO_CENTERING_CALIBRATION_LEFT_SPEED   = -30;
  static const int            AUTO_CENTERING_CALIBRATION_RIGHT_SPEED  =  30;
  static const int            AUTO_CENTERING_CALIBRATION_CENTER_SPEED = -20;
  static const int            AUTO_CENTERING_CALIBRATION_DELAY_MS     =  2000;
  static const int            AUTO_TURN_LEFT_SPEED                    = -80;
  static const int            AUTO_TURN_RIGHT_SPEED                   =  80;
  static const int            AUTO_HALL_SENSOR_COUNT_MAX_DIFF         =  2;
  static const unsigned long  AUTO_MAX_LENGTH_MS                      =  300000;  // Five minutes
  
  // On the Mega, digital pins 2, 3, and 18-21 are interrupts.
  
  // DIGITAL PINS
  static const unsigned int   CH1_INPUT_PIN                           = 2;    // Derby car yaw control
  static const unsigned int   CH2_INPUT_PIN                           = 3;
  static const unsigned int   CH3_INPUT_PIN                           = 4;
  static const unsigned int   CH4_INPUT_PIN                           = 5;    // Recalibrate derby car
  static const unsigned int   CH5_INPUT_PIN                           = 6;    // Derby car brake control
  static const unsigned int   CH6_INPUT_PIN                           = 7;    // Master enable (disable all input control)
  static const unsigned int   STEERING_SPEED_CONTROLLER_PIN           = 8;
  static const unsigned int   BRAKE_SPEED_CONTROLLER_PIN              = 9;
  static const unsigned int   STEERING_LEFT_LIMIT_SWITCH_PIN          = 10;
  static const unsigned int   STEERING_RIGHT_LIMIT_SWITCH_PIN         = 11;
  static const unsigned int   BRAKE_RELEASE_LIMIT_SWITCH_PIN          = 12;
  static const unsigned int   BRAKE_APPLY_LIMIT_SWITCH_PIN            = 13;
  static const unsigned int   STEERING_ENCODER_PIN                    = 14;
  static const unsigned int   SONAR_TRIGGER_PIN                       = 15;
  static const unsigned int   SONAR_ECHO_PIN                          = 16;
  static const unsigned int   LEFT_HALL_SENSOR_PIN                    = 18;   // Must be a board interrupt pin
  static const unsigned int   RIGHT_HALL_SENSOR_PIN                   = 19;   // Must be a board interrupt pin
  static const unsigned int   STEERING_LIMIT_SWITCHES_INTERRUPT_PIN   = 20;   // Must be a board interrupt pin
  static const unsigned int   BRAKE_LIMIT_SWITCHES_INTERRUPT_PIN      = 21;   // Must be a board interrupt pin
  static const unsigned int   AUTONOMOUS_SWITCH_PIN                   = 25;
  static const unsigned int   AUTONOMOUS_LED_PIN                      = 26;
  static const unsigned int   DEBUG_OUTPUT_1_LED_PIN                  = 27;
  static const unsigned int   DEBUG_OUTPUT_2_LED_PIN                  = 28;
  static const unsigned int   DEBUG_OUTPUT_3_LED_PIN                  = 29;
  static const unsigned int   DEBUG_OUTPUT_4_LED_PIN                  = 30;
  
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
  static const int            NUM_MAGNETS_PER_WHEEL                   = 6;
  static const int            POTENTIOMETER_READ_SPACING_DELAY_MS     = 100;
  static const int            POTENTIOMETER_MAX_JITTER_VALUE          = 5;
  static const int            POTENTIOMETER_MAX_VALUE                 = 1024;
  static const int            ENCODER_MAX_VALUE                       = 4096;
  
  // MISC
  static const int            OFF                                     = 0;
  static const int            ON                                      = 100;
  static const unsigned int   TENTH_OF_A_SECOND_DELAY_MS              = 100;
  static const unsigned long  PULSE_IN_TIMEOUT_US                     = 50000;
  
  // DEBUG ASSIST
  static const bool           DEBUG_PRINTS                            = false;
  static const unsigned long  DEBUG_PRINT_INTERVAL_MS                 = 3000;
};

#endif // SOAPBOXDERBYCAR_HPP

