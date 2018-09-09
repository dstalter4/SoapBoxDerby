#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <Servo.h>
#include <EEPROM.h> 

// DIGITAL PINS
static const unsigned int   YAW_INPUT_PIN                           = 2;
static const unsigned int   RECALIBRATE_INPUT_PIN                   = 5;
static const unsigned int   BRAKE_INPUT_PIN                         = 6;
static const unsigned int   MASTER_ENABLE_INPUT_PIN                 = 7;

static const unsigned int   STEERING_SPEED_CONTROLLER_PIN           = 8;
static const unsigned int   BRAKE_MAGNET_RELAY_PIN                  = 9;
static const unsigned int   STEERING_LEFT_LIMIT_SWITCH_PIN          = 10;
static const unsigned int   STEERING_RIGHT_LIMIT_SWITCH_PIN         = 11;

static const unsigned int   LEFT_HALL_SENSOR_PIN                    = 18;   // Must be a board interrupt pin
static const unsigned int   RIGHT_HALL_SENSOR_PIN                   = 19;   // Must be a board interrupt pin
static const unsigned int   STEERING_LIMIT_SWITCHES_INTERRUPT_PIN   = 20;   // Must be a board interrupt pin

static const unsigned int   LEFT_HALL_SENSOR_LED_PIN                = 26;
static const unsigned int   STEER_LIMIT_SWITCHES_LED_PIN            = 27;
static const unsigned int   RIGHT_HALL_SENSOR_LED_PIN               = 28;
static const unsigned int   BRAKE_MAGNET_RELAY_LED_PIN              = 29;
static const unsigned int   AUTONOMOUS_READY_LED_PIN                = 30;
static const unsigned int   EEPROM_RW_LED_PIN                       = 31;
static const unsigned int   DEBUG_OUTPUT_7_LED_PIN                  = 32;
static const unsigned int   DEBUG_OUTPUT_8_LED_PIN                  = 33;
static const unsigned int   DEBUG_OUTPUT_9_LED_PIN                  = 34;
static const unsigned int   AUTONOMOUS_EXECUTING_LED_PIN            = 35;

static const unsigned int   DEBUG_OUTPUT_LEDS_START_PIN             = LEFT_HALL_SENSOR_LED_PIN;
static const unsigned int   DEBUG_OUTPUT_LEDS_END_PIN               = AUTONOMOUS_EXECUTING_LED_PIN;

static const unsigned int   AUTONOMOUS_SWITCH_PIN                   = 44;
static const unsigned int   SERIAL_TRANSMIT_SWITCH_PIN              = 45;
static const unsigned int   SWITCH_3_RESERVED                       = 46;
static const unsigned int   SWITCH_4_RESERVED                       = 47;

static const unsigned int   H_BRIDGE_PIN_1                          = 50;
static const unsigned int   H_BRIDGE_PIN_2                          = 51;
static const unsigned int   H_BRIDGE_PIN_3                          = 52;
static const unsigned int   H_BRIDGE_PIN_4                          = 53;

// ANALOG PINS
static const unsigned int   FRONT_AXLE_POTENTIOMETER_PIN            = 0;

// SERVOS
Servo steeringServo;

// FORWARD DECLARATIONS
void ManualControl();
void LeftHallIsr();
void RightHallIsr();
void SteeringLimitIsr();
void BrakeLimitIsr();
void SerialTest();
void ControllerInputTest();
void SpeedControllerTest();
void BrakeRelayTest();
void LimitSwitchTest();
void SwitchesTest();
void LedTest();
void PotentiometerTest();
void HBridgeTest();
void EepromTest();

void setup()
{
  // controller inputs
  // 2 = yaw, 6 = brake, 7 = master enable
  for (unsigned int i = YAW_INPUT_PIN; i <= MASTER_ENABLE_INPUT_PIN; i++)
  {
    pinMode(i, INPUT);
  }

  // speed controllers
  steeringServo.attach(STEERING_SPEED_CONTROLLER_PIN);
  
  // limit switch wiring test
  pinMode(STEERING_LEFT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEERING_RIGHT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BRAKE_MAGNET_RELAY_PIN, OUTPUT);

  // connect ISR for limit switches
  pinMode(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN), SteeringLimitIsr, CHANGE);

  // connect ISRs for hall sensors
  attachInterrupt(digitalPinToInterrupt(LEFT_HALL_SENSOR_PIN), LeftHallIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_SENSOR_PIN), RightHallIsr, CHANGE);

  // debug outputs
  for (unsigned int i = DEBUG_OUTPUT_LEDS_START_PIN; i <= DEBUG_OUTPUT_LEDS_END_PIN; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // switches test
  pinMode(AUTONOMOUS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SERIAL_TRANSMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SWITCH_3_RESERVED, INPUT_PULLUP);
  pinMode(SWITCH_4_RESERVED, INPUT_PULLUP);
  
  // h-bridge outputs
  pinMode(H_BRIDGE_PIN_1, OUTPUT);
  pinMode(H_BRIDGE_PIN_2, OUTPUT);
  pinMode(H_BRIDGE_PIN_3, OUTPUT);
  pinMode(H_BRIDGE_PIN_4, OUTPUT);
  
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  Serial.println("Testbench");
}

void loop()
{
  ManualControl();
  //SerialTest();
  //ControllerInputTest();
  //SpeedControllerTest();
  //BrakeRelayTest();
  //LimitSwitchTest();
  //SwitchesTest();
  //LedTest();
  //PotentiometerTest();
  //HBridgeTest();
  //EepromTest();
}

void ManualControl()
{
  // Yaw input is roughly 1000 -> 2000, so it can be directly used in writeMicroseconds
  int yawInput = pulseIn(YAW_INPUT_PIN, HIGH, 100000);
  int brakeInput = pulseIn(BRAKE_INPUT_PIN, HIGH, 100000);
  int left = digitalRead(STEERING_LEFT_LIMIT_SWITCH_PIN);
  int right = digitalRead(STEERING_RIGHT_LIMIT_SWITCH_PIN);

  if (brakeInput < 1350)
  {
    digitalWrite(BRAKE_MAGNET_RELAY_PIN, HIGH);
    digitalWrite(BRAKE_MAGNET_RELAY_LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(BRAKE_MAGNET_RELAY_PIN, LOW);
    digitalWrite(BRAKE_MAGNET_RELAY_LED_PIN, LOW);
  }

  // Controller is off, just return
  if (yawInput == 0)
  {
    return;
  }

  // Use some default minimum thresholds of 1425 and 1575
  if ((yawInput < 1425) && (left != 1))
  {
    //Serial.println("L");
    steeringServo.writeMicroseconds(yawInput);
  }
  else if ((yawInput > 1575) && (right != 1))
  {
    //Serial.println("R");
    steeringServo.writeMicroseconds(yawInput);
  }
  else
  {
    // Off
    //Serial.println("OFF");
    steeringServo.writeMicroseconds(1500);
  }
}

void LeftHallIsr()
{
  // get pin status to determine if rising or falling edge
  int interruptEdge = digitalRead(LEFT_HALL_SENSOR_PIN);
  
  // debug LED on
  digitalWrite(LEFT_HALL_SENSOR_LED_PIN, !interruptEdge);
}

void RightHallIsr()
{
  // get pin status to determine if rising or falling edge
  int interruptEdge = digitalRead(RIGHT_HALL_SENSOR_PIN);
  
  // debug LED on
  digitalWrite(RIGHT_HALL_SENSOR_LED_PIN, !interruptEdge);
}

void SteeringLimitIsr()
{
  // reverse the LED logic since this is falling edge
  int ledState = (digitalRead(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN) == 1) ? LOW : HIGH;
  digitalWrite(STEER_LIMIT_SWITCHES_LED_PIN, ledState);
}

void SerialTest()
{
  struct SerialData
  {
    int32_t steerMotorSpeed;
    int32_t brakeApplied;
    int32_t leftHallVal;
    int32_t rightHallVal;
    int32_t leftHallCount;
    int32_t rightHallCount;
    int32_t leftLimit;
    int32_t rightLimit;
    int32_t potentiometer;
    int32_t inAuto;
  };
  
  //static SerialData serialData = {95,1,12345,-7,2,-18,3,0,-58,987};
  //static SerialData serialData = {64,65,66,67,68,69,70,71,72,73};
  //static SerialData serialData = {255,0,255,0,255,0,255,0,255,0};
  static SerialData serialData = {0,0,0,1,3,3,1,0,340,0};
  //byte * pData = reinterpret_cast<byte*>(&serialData);

  if (Serial3.available())
  {
    String s = Serial3.readString();
    if (s == "pi")
    {
      Serial.println("Got pi.");
    }
    else
    {
      Serial.print("Not pi: ");
      Serial.println(s);
      return;
    }
  }
  else
  {
    return;
  }
  
  static int x = 0;
  Serial.print("SerialTest: ");
  Serial.println(++x);

  Serial3.println("SBDC");
  Serial3.println(serialData.steerMotorSpeed);
  Serial3.println(serialData.brakeApplied);
  Serial3.println(serialData.leftHallVal);
  Serial3.println(serialData.rightHallVal);
  Serial3.println(serialData.leftHallCount);
  Serial3.println(serialData.rightHallCount);
  Serial3.println(serialData.leftLimit);
  Serial3.println(serialData.rightLimit);
  Serial3.println(serialData.potentiometer);
  Serial3.println(serialData.inAuto);

  if (serialData.steerMotorSpeed >= 0)
  {
    serialData.steerMotorSpeed += 5;
    if (serialData.steerMotorSpeed == 100)
    {
      serialData.steerMotorSpeed = -5;
    }
  }
  else
  {
    serialData.steerMotorSpeed -= 5;
    if (serialData.steerMotorSpeed == -100)
    {
      serialData.steerMotorSpeed = 0;
    }
  }

  serialData.brakeApplied = !static_cast<bool>(serialData.brakeApplied);
  serialData.leftHallVal = !static_cast<bool>(serialData.leftHallVal);
  serialData.rightHallVal = !static_cast<bool>(serialData.rightHallVal);

  serialData.leftHallCount++;
  serialData.rightHallCount++;

  switch (serialData.potentiometer)
  {
    case 340:
    {
      serialData.potentiometer = 325;
      break;
    }
    case 325:
    {
      serialData.potentiometer = 355;
      break;
    }
    case 355:
    default:
    {
      serialData.potentiometer = 340;
      break;
    }
  }

  serialData.inAuto = !static_cast<bool>(serialData.inAuto);
  
  /*
  // This prints raw data with a byte pointer
  for (size_t x = 0; x < sizeof(SerialData); x++)
  {
    Serial.print(*pData, HEX);
    Serial.print(" ");
    Serial3.write(*pData++);
  }
  Serial3.println();
  */

  // Display any data that Serial3 receives
  while (Serial2.available())
  {
    Serial.print(Serial2.read());
  }
  
  Serial.println();
  //delay(1000);
}

void ControllerInputTest()
{
  int yawInput = pulseIn(YAW_INPUT_PIN, HIGH, 100000);
  int recalibrateInput = pulseIn(RECALIBRATE_INPUT_PIN, HIGH, 100000);
  int brakeInput = pulseIn(BRAKE_INPUT_PIN, HIGH, 100000);
  int emergencyStopInput = pulseIn(MASTER_ENABLE_INPUT_PIN, HIGH, 100000);

  Serial.println("Yaw, Recalibrate, Brake, E-Stop");
  Serial.print(yawInput);
  Serial.print(", ");
  Serial.print(recalibrateInput);
  Serial.print(", ");
  Serial.print(brakeInput);
  Serial.print(", ");
  Serial.println(emergencyStopInput);
  delay(1000);
}

void SpeedControllerTest()
{
  // 1000us = full reverse
  // 1500us = neutral
  // 2000us = full forward
  const int PWM_SCALE_FACTOR = 5;
  const int PWM_BASE_VALUE = 1500;
  
/*
  bool bStart = false;
  while (!bStart)
  {
    int value = 0;
    if (Serial.available())
    {
      value = Serial.parseInt();
      Serial.print("Got value: ");
      Serial.println(value);
    }

    if (value == 1500)
    {
      Serial.println("Starting...");
      bStart = true;
    }
  }
*/

  Serial.println(1750);
  steeringServo.writeMicroseconds(1750);
  delay(1500);
  Serial.println(2000);
  steeringServo.writeMicroseconds(2000);
  delay(1500);
  Serial.println(1500);
  steeringServo.writeMicroseconds(1500);
  delay(1500);
  Serial.println(1250);
  steeringServo.writeMicroseconds(1250);
  delay(1500);
  Serial.println(1000);
  steeringServo.writeMicroseconds(1000);
  delay(1500);
  Serial.println(1500);
  steeringServo.writeMicroseconds(1500);
  delay(1500);
  return;

  // ramp up from neutral to full forward
  for (int value = 0; value <= 100; value++)
  {
    steeringServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    delay(100);
  }

  // motor off
  steeringServo.writeMicroseconds(1500);
  delay(1500);

  // ramp down from neutral to full reverse
  for (int value = 0; value >= -100; value--)
  {
    steeringServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    delay(100);
  }
  return;
}

void BrakeRelayTest()
{
  Serial.println("Relay on.");
  digitalWrite(BRAKE_MAGNET_RELAY_PIN, HIGH);
  digitalWrite(BRAKE_MAGNET_RELAY_LED_PIN, HIGH);
  delay(3000);
  Serial.println("Relay off.");
  digitalWrite(BRAKE_MAGNET_RELAY_PIN, LOW);
  digitalWrite(BRAKE_MAGNET_RELAY_LED_PIN, LOW);
  delay(3000);
}

void LimitSwitchTest()
{
  int left = digitalRead(STEERING_LEFT_LIMIT_SWITCH_PIN);
  int right = digitalRead(STEERING_RIGHT_LIMIT_SWITCH_PIN);
  Serial.print(left);
  Serial.print(", ");
  Serial.println(right);
  delay(500);
}

void SwitchesTest()
{
  int autoSwitch = digitalRead(AUTONOMOUS_SWITCH_PIN);
  digitalWrite(AUTONOMOUS_READY_LED_PIN, autoSwitch);
  Serial.print("Auto switch: ");
  Serial.println(autoSwitch);
  int serialTransmitSwitch = digitalRead(SERIAL_TRANSMIT_SWITCH_PIN);
  digitalWrite(EEPROM_RW_LED_PIN, serialTransmitSwitch);
  Serial.print("Serial switch: ");
  Serial.println(serialTransmitSwitch);
  int switch3 = digitalRead(SWITCH_3_RESERVED);
  digitalWrite(LEFT_HALL_SENSOR_LED_PIN, switch3);
  Serial.print("Switch 3: ");
  Serial.println(switch3);
  int switch4 = digitalRead(SWITCH_4_RESERVED);
  digitalWrite(RIGHT_HALL_SENSOR_LED_PIN, switch4);
  Serial.print("Switch 4: ");
  Serial.println(switch4);
  Serial.println();
  delay(500);
}

void LedTest()
{
  for (unsigned int i = DEBUG_OUTPUT_LEDS_START_PIN; i <= DEBUG_OUTPUT_LEDS_END_PIN; i++)
  {
    // First turn them on
    digitalWrite(i, HIGH);
    delay(500);
  }

  for (unsigned int i = DEBUG_OUTPUT_LEDS_START_PIN; i <= DEBUG_OUTPUT_LEDS_END_PIN; i++)
  {
    // Then turn them off
    digitalWrite(i, LOW);
    delay(500);
  }
}

void PotentiometerTest()
{
  // Keyes KY-040 Rotary Encoder
  int pot = analogRead(FRONT_AXLE_POTENTIOMETER_PIN);
  Serial.println(pot);
  delay(500);
}

void HBridgeTest()
{
  // put your main code here, to run repeatedly:

  /*
  while (true)
  {
    analogWrite(H_BRIDGE_PIN_1, 255);
    analogWrite(H_BRIDGE_PIN_2, 255);
    analogWrite(H_BRIDGE_PIN_3, 255);
    analogWrite(H_BRIDGE_PIN_4, 255);
  }
  */
  
  for (int i = 0; i <= 255; i++)
  {
    //analogWrite(H_BRIDGE_PIN_1, i);
    analogWrite(H_BRIDGE_PIN_2, i);
    //analogWrite(H_BRIDGE_PIN_3, i);
    //analogWrite(H_BRIDGE_PIN_4, i);
    delay(100);
  }

  delay(3000);

  for (int i = 255; i >= 0; i--)
  {
    //analogWrite(H_BRIDGE_PIN_1, i);
    analogWrite(H_BRIDGE_PIN_2, i);
    //analogWrite(H_BRIDGE_PIN_3, i);
    //analogWrite(H_BRIDGE_PIN_4, i);
    delay(100);
  }
}

void EepromTest()
{
  if (Serial.available())
  {
    switch (Serial.read())
    {
      case 'w':
      {
        // Empirical measurement, writing all of EEPROM takes 14 seconds
        
        unsigned long writeStartTime = millis();
        
        for (size_t i = 0; i < EEPROM.length(); i++)
        {
          EEPROM.write(i, i);
        }

        unsigned long writeEndTime = millis();
        
        Serial.print("Write start time: ");
        Serial.println(writeStartTime);
        Serial.print("Write end time: ");
        Serial.println(writeEndTime);
        Serial.print("Write time: ");
        Serial.println(writeEndTime - writeStartTime);
        Serial.println();
        Serial.println();
        
        break;
      }
      case 'r':
      {
        // Empirical measurement, reading all of EEPROM takes 10 milliseconds
        Serial.println("Enter offset (512B chunks).");
        while (!Serial.available())
        {
        }

        int offset = Serial.read() - '0';
        if (offset >= 8)
        {
          Serial.println("Invalid EEPROM offset.");
          break;
        }
        Serial.print("Reading from 512B offset: ");
        Serial.println(offset);
        
        unsigned long readStartTime = millis();

        // EEPROM on Mega is 4kB, => chunk it into 8 x 512B for displaying
        const size_t EEPROM_CHUNK_SIZE = EEPROM.length() / 8;
        byte data[EEPROM_CHUNK_SIZE];
        for (size_t i = 0; i < EEPROM_CHUNK_SIZE; i++)
        {
          data[i] = EEPROM.read(i + (offset * EEPROM_CHUNK_SIZE));
        }
        
        unsigned long readEndTime = millis();

        for (size_t i = 0; i < EEPROM_CHUNK_SIZE; i++)
        {
          Serial.print("Address ");
          Serial.print(i + (offset * EEPROM_CHUNK_SIZE), HEX);
          Serial.print(": ");
          Serial.println(data[i]);
        }

        Serial.println();
        Serial.print("Read start time: ");
        Serial.println(readStartTime);
        Serial.print("Read end time: ");
        Serial.println(readEndTime);
        Serial.print("Read time: ");
        Serial.println(readEndTime - readStartTime);
        Serial.println();
        Serial.println();
        
        break;
      }
      default:
      {
        break;
      }
    }
  }
}

