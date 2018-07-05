#include <Servo.h>

// DIGITAL PINS
static const unsigned int   YAW_INPUT_PIN                           = 2;
static const unsigned int   RECALIBRATE_INPUT_PIN                   = 5;
static const unsigned int   BRAKE_INPUT_PIN                         = 6;
static const unsigned int   MASTER_ENABLE_INPUT_PIN                 = 7;
static const unsigned int   STEERING_SPEED_CONTROLLER_PIN           = 8;
static const unsigned int   BRAKE_SPEED_CONTROLLER_PIN              = 9;
static const unsigned int   STEERING_LEFT_LIMIT_SWITCH_PIN          = 10;
static const unsigned int   STEERING_RIGHT_LIMIT_SWITCH_PIN         = 11;
static const unsigned int   BRAKE_RELEASE_LIMIT_SWITCH_PIN          = 12;
static const unsigned int   BRAKE_APPLY_LIMIT_SWITCH_PIN            = 13;
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

// Servos for speed controller test
Servo steeringServo;
Servo brakingServo;

void SerialTest();
void ControllerInputTest();
void SpeedControllerTest();
void SteeringLimitIsr();
void BrakeLimitIsr();
void LimitSwitchTest();
void AutoSwitchTest();
void LeftHallIsr();
void RightHallIsr();
void PotentiometerTest();
void HBridgeTest();

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
  brakingServo.attach(BRAKE_SPEED_CONTROLLER_PIN);
  
  // limit switch wiring test
  pinMode(STEERING_LEFT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEERING_RIGHT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BRAKE_RELEASE_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BRAKE_APPLY_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // connect ISRs for limit switches
  attachInterrupt(digitalPinToInterrupt(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN), SteeringLimitIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BRAKE_LIMIT_SWITCHES_INTERRUPT_PIN), BrakeLimitIsr, CHANGE);

  // auto switch test
  pinMode(AUTONOMOUS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(AUTONOMOUS_LED_PIN, OUTPUT);

  // connect ISRs for hall sensors
  attachInterrupt(digitalPinToInterrupt(LEFT_HALL_SENSOR_PIN), LeftHallIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_SENSOR_PIN), RightHallIsr, CHANGE);

  // debug outputs
  pinMode(DEBUG_OUTPUT_1_LED_PIN, OUTPUT);
  pinMode(DEBUG_OUTPUT_2_LED_PIN, OUTPUT);
  pinMode(DEBUG_OUTPUT_3_LED_PIN, OUTPUT);
  pinMode(DEBUG_OUTPUT_4_LED_PIN, OUTPUT);
  
  // h-bridge outputs
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
}

void loop()
{
  SerialTest();
  //ControllerInputTest();
  //SpeedControllerTest();
  //LimitSwitchTest();
  //AutoSwitchTest();
  //PotentiometerTest();
  //HBridgeTest();
}

void LeftHallIsr()
{
  // get pin status to determine if rising or falling edge
  int interruptEdge = digitalRead(LEFT_HALL_SENSOR_PIN);
  
  // debug LED on
  digitalWrite(DEBUG_OUTPUT_1_LED_PIN, interruptEdge);
}

void RightHallIsr()
{
  // get pin status to determine if rising or falling edge
  int interruptEdge = digitalRead(RIGHT_HALL_SENSOR_PIN);
  
  // debug LED on
  digitalWrite(DEBUG_OUTPUT_2_LED_PIN, interruptEdge);
}

void SteeringLimitIsr()
{
  // reverse the LED logic since this is falling edge
  int ledState = (digitalRead(STEERING_LIMIT_SWITCHES_INTERRUPT_PIN) == 1) ? LOW : HIGH;
  digitalWrite(DEBUG_OUTPUT_3_LED_PIN, ledState);
}

void BrakeLimitIsr()
{
  // reverse the LED logic since this is falling edge
  int ledState = (digitalRead(BRAKE_LIMIT_SWITCHES_INTERRUPT_PIN) == 1) ? LOW : HIGH;
  digitalWrite(DEBUG_OUTPUT_4_LED_PIN, ledState);
}

void SerialTest()
{
  struct SerialData
  {
    int32_t steerMotorSpeed;
    int32_t brakeRelease;
    int32_t brakeApply;
    int32_t leftHallVal;
    int32_t rightHallVal;
    int32_t leftHallCount;
    int32_t rightHallCount;
    int32_t leftLimit;
    int32_t rightLimit;
    int32_t brakeReleaseLimit;
    int32_t brakeApplyLimit;
    int32_t potentiometer;
    int32_t inAuto;
  };
  
  static SerialData serialData = {95,1,12345,340,-7,2,-18,3,0,4,0,-58,987};
  //static SerialData serialData = {64,65,66,67,68,69,70,71,72,73,74,75,76};
  //static SerialData serialData = {255,0,255,0,255,0,255,0,255,0,255,0,255};
  //byte * pData = reinterpret_cast<byte*>(&serialData);
  
  static int x = 0;
  Serial.print("SerialTest: ");
  Serial.println(x++);

  Serial3.println("SBDC");
  Serial3.println(serialData.steerMotorSpeed);
  Serial3.println(serialData.brakeRelease);
  Serial3.println(serialData.brakeApply);
  Serial3.println(serialData.leftHallVal);
  Serial3.println(serialData.rightHallVal);
  Serial3.println(serialData.leftHallCount);
  Serial3.println(serialData.rightHallCount);
  Serial3.println(serialData.leftLimit);
  Serial3.println(serialData.rightLimit);
  Serial3.println(serialData.brakeReleaseLimit);
  Serial3.println(serialData.brakeApplyLimit);
  Serial3.println(serialData.potentiometer);
  Serial3.println(serialData.inAuto);
  
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
  while (Serial3.available())
  {
    Serial.print(Serial3.read());
  }
  
  Serial.println();
  delay(3000);

  /*
  // Old test
  if (Serial)
  {
    digitalWrite(DEBUG_OUTPUT_1_LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(DEBUG_OUTPUT_2_LED_PIN, HIGH);
  }
  */
}

void ControllerInputTest()
{
  int in3 = pulseIn(3, HIGH, 100000);
  int in4 = pulseIn(4, HIGH, 100000);
  int in5 = pulseIn(5, HIGH, 100000);
  Serial.println(in3);
  Serial.println(in4);
  Serial.println(in5);
  Serial.println();
  delay(2000);
  return;
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
  for (int i = 700; i < 2000; i++)
  {
    steeringServo.writeMicroseconds(i);
    delay(10);
    if ((i % 100) == 0)
    {
      Serial.println(i);
    }
  }
  steeringServo.writeMicroseconds(1350);
  delay(1000);
  return;
*/
/*
  delay(3000);
  steeringServo.writeMicroseconds(2000);
  delay(3000);
  steeringServo.writeMicroseconds(1500);
  delay(1000);
  steeringServo.writeMicroseconds(1000);
  delay(1000);
  steeringServo.writeMicroseconds(1500);
  delay(1000);
  return;
*/
/*
  static int value = 0;
  if (Serial.available())
  {
    value = Serial.parseInt();
  }
  if (value != 0)
  {
    Serial.print("Writing: ");
    Serial.println(value);
    steeringServo.writeMicroseconds(value);
  }
  delay(1000);
  return;
*/
  static bool bStart = false;
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
  
  for (int value = 0; value <= 100; value++)
  {
    steeringServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    delay(100);
  }
  steeringServo.writeMicroseconds(1500);
  delay(1500);
  for (int value = 0; value >= -100; value--)
  {
    steeringServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    delay(100);
  }
  return;
      
  // ramp up from full reverse to full forward
  for (int value = -20; value <= 20; value++)
  {
    steeringServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    brakingServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    delay(100);
  }

  // ramp down from full forward to full reverse
  for (int value = 20; value >= -20; value--)
  {
    steeringServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    brakingServo.writeMicroseconds((value * PWM_SCALE_FACTOR) + PWM_BASE_VALUE);
    delay(100);
  }
}

void LimitSwitchTest()
{
  int left = digitalRead(STEERING_LEFT_LIMIT_SWITCH_PIN);
  int right = digitalRead(STEERING_RIGHT_LIMIT_SWITCH_PIN);
  int brake1 = digitalRead(BRAKE_RELEASE_LIMIT_SWITCH_PIN);
  int brake2 = digitalRead(BRAKE_APPLY_LIMIT_SWITCH_PIN);
  Serial.print(left);
  Serial.print(", ");
  Serial.print(right);
  Serial.print(", ");
  Serial.print(brake1);
  Serial.print(", ");
  Serial.println(brake2);
  delay(500);
}

void AutoSwitchTest()
{
  int autoSwitch = digitalRead(AUTONOMOUS_SWITCH_PIN);
  digitalWrite(AUTONOMOUS_LED_PIN, autoSwitch);
  Serial.println(autoSwitch);
  delay(500);
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

  while (true)
  {
    //analogWrite(22, 255);
    analogWrite(23, 255);
    //analogWrite(24, 255);
    //analogWrite(25, 255);
  }
  
  for (int i = 0; i <= 255; i++)
  {
    //analogWrite(22, i);
    analogWrite(23, i);
    //analogWrite(24, i);
    //analogWrite(25, i);
    delay(100);
  }

  delay(3000);

  for (int i = 255; i >= 0; i--)
  {
    //analogWrite(22, i);
    analogWrite(23, i);
    //analogWrite(24, i);
    //analogWrite(25, i);
    delay(100);
  }
}

