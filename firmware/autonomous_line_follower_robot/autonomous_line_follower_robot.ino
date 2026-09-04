#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

/*
  Autonomous Line-Following and Obstacle-Avoidance Robot

  Consolidated final sketch reconstructed from the supplied project material:
  - Arduino Mega 2560
  - Five digital IR line sensors
  - L298N dual H-bridge and two DC motor channels
  - HC-SR04 ultrasonic sensor mounted on a servo
  - TCS34725 RGB color sensor

  Arduino Mega timer note:
  L298N ENA and ENB are assigned to PWM-capable pins 6 and 11. These pins remain
  available when the Servo library drives the scanning servo on the Mega 2560.

  Validation note:
  Confirm the pin mapping on the physical robot and calibrate the PD gains,
  color thresholds, servo angles, and timed detour values before operation.
*/

// ============================================================================
// Pin assignments
// ============================================================================

// L298N direction pins
const uint8_t LEFT_MOTOR_IN1  = 2;
const uint8_t LEFT_MOTOR_IN2  = 3;
const uint8_t RIGHT_MOTOR_IN1 = 4;
const uint8_t RIGHT_MOTOR_IN2 = 5;

// L298N speed-control pins (remove the ENA/ENB jumpers when using PWM)
const uint8_t LEFT_MOTOR_ENABLE  = 6;
const uint8_t RIGHT_MOTOR_ENABLE = 11;

// Five IR sensors ordered from the robot's left side to its right side
const uint8_t LINE_SENSOR_PINS[5] = {7, 8, 9, 10, 12};
const int8_t LINE_SENSOR_WEIGHTS[5] = {-4, -2, 0, 2, 4};

// Most digital IR modules output LOW when positioned over a black line.
// Change LOW to HIGH if the readings are reversed on the physical module.
const uint8_t LINE_DETECTED_STATE = LOW;

// HC-SR04 and scanning servo
const uint8_t ULTRASONIC_TRIG_PIN = 13;
const uint8_t ULTRASONIC_ECHO_PIN = A0;
const uint8_t SCAN_SERVO_PIN      = A1;

// TCS34725 uses the Arduino Mega 2560 I2C pins: SDA=20 and SCL=21.

// ============================================================================
// Adjustable motion values
// ============================================================================

const int BASE_LINE_SPEED = 145;
const int MAX_LINE_SPEED  = 220;
const int SEARCH_SPEED    = 120;
const int TURN_SPEED      = 160;
const int DETOUR_SPEED    = 150;

// Simple PD steering controller for the five line sensors
const float LINE_KP = 28.0;
const float LINE_KD = 18.0;

// Stop approximately 10-15 cm before an obstacle.
const int OBSTACLE_DISTANCE_CM = 15;

// These timings depend on the motors, battery voltage, wheels, and chassis.
const unsigned long QUARTER_TURN_TIME_MS = 430;
const unsigned long SIDE_STEP_TIME_MS    = 650;
const unsigned long PASS_OBSTACLE_TIME_MS = 950;
const unsigned long FIND_LINE_TIMEOUT_MS  = 2200;

// Servo angles may need to be swapped if the physical mounting is reversed.
const int SERVO_CENTER_ANGLE = 90;
const int SERVO_LEFT_ANGLE   = 150;
const int SERVO_RIGHT_ANGLE  = 30;

// ============================================================================
// Color-marker settings
// ============================================================================

enum MarkerColor {
  NO_MARKER,
  RED_MARKER,
  GREEN_MARKER,
  BLUE_MARKER
};

Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_1X
);

const unsigned long RED_STOP_TIME_MS   = 5000;
const unsigned long GREEN_STOP_TIME_MS = 5000;
const unsigned long BLUE_STOP_TIME_MS  = 2000;

const unsigned long COLOR_CHECK_INTERVAL_MS = 150;
const uint16_t MINIMUM_CLEAR_READING = 100;

Servo scanServo;

bool colorSensorAvailable = false;
bool currentMarkerHandled = false;
bool lineRecoveryRequired = false;

int lastLineError = 0;
unsigned long lastDistanceCheckMs = 0;
unsigned long lastColorCheckMs = 0;

// ============================================================================
// Motor control
// ============================================================================

void setOneMotor(
  uint8_t input1,
  uint8_t input2,
  uint8_t enablePin,
  int requestedSpeed
) {
  int motorSpeed = constrain(requestedSpeed, -255, 255);

  if (motorSpeed > 0) {
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
  } else {
    digitalWrite(input1, LOW);
    digitalWrite(input2, LOW);
  }

  analogWrite(enablePin, abs(motorSpeed));
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setOneMotor(
    LEFT_MOTOR_IN1,
    LEFT_MOTOR_IN2,
    LEFT_MOTOR_ENABLE,
    leftSpeed
  );

  setOneMotor(
    RIGHT_MOTOR_IN1,
    RIGHT_MOTOR_IN2,
    RIGHT_MOTOR_ENABLE,
    rightSpeed
  );
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

void driveFor(int leftSpeed, int rightSpeed, unsigned long durationMs) {
  setMotorSpeeds(leftSpeed, rightSpeed);
  delay(durationMs);
  stopMotors();
  delay(80);
}

void turnLeftFor(unsigned long durationMs) {
  driveFor(-TURN_SPEED, TURN_SPEED, durationMs);
}

void turnRightFor(unsigned long durationMs) {
  driveFor(TURN_SPEED, -TURN_SPEED, durationMs);
}

// ============================================================================
// Line following
// ============================================================================

bool lineIsDetected(int &calculatedError) {
  int weightedTotal = 0;
  int detectedSensorCount = 0;

  for (uint8_t index = 0; index < 5; index++) {
    bool sensorOnLine =
      digitalRead(LINE_SENSOR_PINS[index]) == LINE_DETECTED_STATE;

    if (sensorOnLine) {
      weightedTotal += LINE_SENSOR_WEIGHTS[index];
      detectedSensorCount++;
    }
  }

  if (detectedSensorCount == 0) {
    return false;
  }

  calculatedError = weightedTotal / detectedSensorCount;
  return true;
}

bool anyLineSensorIsActive() {
  for (uint8_t index = 0; index < 5; index++) {
    if (digitalRead(LINE_SENSOR_PINS[index]) == LINE_DETECTED_STATE) {
      return true;
    }
  }

  return false;
}

void recoverLostLine() {
  if (lastLineError < 0) {
    // The line was last seen on the left.
    setMotorSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
  } else if (lastLineError > 0) {
    // The line was last seen on the right.
    setMotorSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
  } else {
    // Move slowly across a short gap when no direction is known yet.
    setMotorSpeeds(SEARCH_SPEED, SEARCH_SPEED);
  }
}

void followLine() {
  int currentError = 0;

  if (!lineIsDetected(currentError)) {
    recoverLostLine();
    return;
  }

  int errorChange = currentError - lastLineError;
  int steeringCorrection = (int)(
    LINE_KP * currentError + LINE_KD * errorChange
  );

  // A positive error means that the line is toward the robot's right side.
  int leftSpeed  = BASE_LINE_SPEED + steeringCorrection;
  int rightSpeed = BASE_LINE_SPEED - steeringCorrection;

  leftSpeed = constrain(leftSpeed, -MAX_LINE_SPEED, MAX_LINE_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_LINE_SPEED, MAX_LINE_SPEED);

  setMotorSpeeds(leftSpeed, rightSpeed);
  lastLineError = currentError;
}

// ============================================================================
// Ultrasonic sensing and obstacle avoidance
// ============================================================================

long measureDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long pulseDuration = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    25000UL
  );

  if (pulseDuration == 0) {
    return 400;
  }

  return (long)(pulseDuration * 0.0343 / 2.0);
}

long averageDistanceCm() {
  long totalDistance = 0;

  for (uint8_t sample = 0; sample < 3; sample++) {
    totalDistance += measureDistanceCm();
    delay(25);
  }

  return totalDistance / 3;
}

long lookAndMeasure(int servoAngle) {
  scanServo.write(servoAngle);
  delay(350);
  return averageDistanceCm();
}

bool obstacleIsAhead() {
  unsigned long currentTime = millis();

  if (currentTime - lastDistanceCheckMs < 80) {
    return false;
  }

  lastDistanceCheckMs = currentTime;
  long forwardDistance = measureDistanceCm();

  return forwardDistance <= OBSTACLE_DISTANCE_CM;
}

bool driveTowardLine(unsigned long timeoutMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < timeoutMs) {
    setMotorSpeeds(DETOUR_SPEED, DETOUR_SPEED);

    // Ignore the starting edge briefly before looking for the return line.
    if (millis() - startTime > 180 && anyLineSensorIsActive()) {
      stopMotors();
      delay(100);
      return true;
    }

    delay(5);
  }

  stopMotors();
  return false;
}

bool performLeftDetour() {
  turnLeftFor(QUARTER_TURN_TIME_MS);
  driveFor(DETOUR_SPEED, DETOUR_SPEED, SIDE_STEP_TIME_MS);
  turnRightFor(QUARTER_TURN_TIME_MS);
  driveFor(DETOUR_SPEED, DETOUR_SPEED, PASS_OBSTACLE_TIME_MS);
  turnRightFor(QUARTER_TURN_TIME_MS);
  bool lineRecovered = driveTowardLine(FIND_LINE_TIMEOUT_MS);

  if (lineRecovered) {
    turnLeftFor(QUARTER_TURN_TIME_MS);
  }

  return lineRecovered;
}

bool performRightDetour() {
  turnRightFor(QUARTER_TURN_TIME_MS);
  driveFor(DETOUR_SPEED, DETOUR_SPEED, SIDE_STEP_TIME_MS);
  turnLeftFor(QUARTER_TURN_TIME_MS);
  driveFor(DETOUR_SPEED, DETOUR_SPEED, PASS_OBSTACLE_TIME_MS);
  turnLeftFor(QUARTER_TURN_TIME_MS);
  bool lineRecovered = driveTowardLine(FIND_LINE_TIMEOUT_MS);

  if (lineRecovered) {
    turnRightFor(QUARTER_TURN_TIME_MS);
  }

  return lineRecovered;
}

void avoidObstacle() {
  stopMotors();
  Serial.println("Obstacle detected");
  delay(150);

  long leftDistance = lookAndMeasure(SERVO_LEFT_ANGLE);
  long rightDistance = lookAndMeasure(SERVO_RIGHT_ANGLE);

  scanServo.write(SERVO_CENTER_ANGLE);
  delay(250);

  Serial.print("Left distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm | Right distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  bool lineRecovered = false;

  if (leftDistance >= rightDistance) {
    Serial.println("Bypassing obstacle on the left");
    lineRecovered = performLeftDetour();
  } else {
    Serial.println("Bypassing obstacle on the right");
    lineRecovered = performRightDetour();
  }

  scanServo.write(SERVO_CENTER_ANGLE);
  lineRecoveryRequired = !lineRecovered;

  if (lineRecovered) {
    lastLineError = 0;
    Serial.println("Line reacquired");
  } else {
    stopMotors();
    Serial.println("Line not found; entering controlled recovery");
  }

  lastDistanceCheckMs = millis();
}

// ============================================================================
// TCS34725 colored-marker detection
// ============================================================================

bool stronglyDominates(uint16_t value, uint16_t other1, uint16_t other2) {
  // The selected channel must be at least 30% stronger than both alternatives.
  return (uint32_t)value * 10UL > (uint32_t)other1 * 13UL &&
         (uint32_t)value * 10UL > (uint32_t)other2 * 13UL;
}

MarkerColor readMarkerColor() {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t clearChannel;

  colorSensor.getRawData(&red, &green, &blue, &clearChannel);

  if (clearChannel < MINIMUM_CLEAR_READING) {
    return NO_MARKER;
  }

  if (stronglyDominates(red, green, blue)) {
    return RED_MARKER;
  }

  if (stronglyDominates(green, red, blue)) {
    return GREEN_MARKER;
  }

  if (stronglyDominates(blue, red, green)) {
    return BLUE_MARKER;
  }

  return NO_MARKER;
}

void stopForMarker(MarkerColor markerColor) {
  stopMotors();

  switch (markerColor) {
    case RED_MARKER:
      Serial.println("Red marker: stop for 5 seconds");
      delay(RED_STOP_TIME_MS);
      // The recovered project mentioned unloading here, but no actuator or
      // mechanism was documented, so no invented unloading motion is added.
      break;

    case GREEN_MARKER:
      Serial.println("Green marker: stop for 5 seconds");
      delay(GREEN_STOP_TIME_MS);
      break;

    case BLUE_MARKER:
      Serial.println("Blue marker: stop for 2 seconds");
      delay(BLUE_STOP_TIME_MS);
      break;

    case NO_MARKER:
      break;
  }
}

bool handleColorMarkerIfNeeded() {
  if (!colorSensorAvailable) {
    return false;
  }

  unsigned long currentTime = millis();

  if (currentTime - lastColorCheckMs < COLOR_CHECK_INTERVAL_MS) {
    return false;
  }

  lastColorCheckMs = currentTime;
  MarkerColor markerColor = readMarkerColor();

  if (markerColor == NO_MARKER) {
    currentMarkerHandled = false;
    return false;
  }

  if (currentMarkerHandled) {
    return false;
  }

  currentMarkerHandled = true;
  stopForMarker(markerColor);
  return true;
}

// ============================================================================
// Arduino setup and main control loop
// ============================================================================

void setup() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  for (uint8_t index = 0; index < 5; index++) {
    pinMode(LINE_SENSOR_PINS[index], INPUT);
  }

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  stopMotors();
  Serial.begin(9600);

  scanServo.attach(SCAN_SERVO_PIN);
  scanServo.write(SERVO_CENTER_ANGLE);
  delay(500);

  colorSensorAvailable = colorSensor.begin();

  if (colorSensorAvailable) {
    Serial.println("TCS34725 color sensor detected");
  } else {
    Serial.println("TCS34725 not detected; continuing without color markers");
  }

  Serial.println("Robot ready");
}

void loop() {
  // Collision avoidance has the highest priority.
  if (obstacleIsAhead()) {
    avoidObstacle();
    return;
  }

  // If a timed detour ended without finding the track, search cautiously
  // instead of applying the final turn and continuing blindly.
  if (lineRecoveryRequired) {
    if (anyLineSensorIsActive()) {
      stopMotors();
      lineRecoveryRequired = false;
      lastLineError = 0;
      Serial.println("Line recovered after detour timeout");
      delay(100);
    } else {
      recoverLostLine();
    }

    return;
  }

  // Colored checkpoints have priority over normal line following.
  if (handleColorMarkerIfNeeded()) {
    return;
  }

  followLine();
}
