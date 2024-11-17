#include <SpeedyStepper.h>
#include <Servo.h>
#include <Arduino.h>
//#include <MPU6050_light.h>
#include <Wire.h>

// Pin assignments for RAMPS 1.4
const int MOTOR_X_STEP_PIN = 54;
const int MOTOR_X_DIRECTION_PIN = 55;
const int X_ENABLE_PIN = 38;

const int MOTOR_Y_STEP_PIN = 60;
const int MOTOR_Y_DIRECTION_PIN = 61;
const int Y_ENABLE_PIN = 56;

const int MOTOR_Z_STEP_PIN = 46;
const int MOTOR_Z_DIRECTION_PIN = 48;
const int Z_ENABLE_PIN = 62;

const int MOTOR_E_STEP_PIN = 26; // E0
const int MOTOR_E_DIRECTION_PIN = 28; // E0
const int E0_ENABLE_PIN = 24; // E0

const int MOTOR_E1_STEP_PIN = 36; // E1
const int MOTOR_E1_DIRECTION_PIN = 34; // E1
const int E1_ENABLE_PIN = 30; // E1

const int MOTOR_E2_STEP_PIN = 6;
const int MOTOR_E2_DIRECTION_PIN = 5;
const int E2_ENABLE_PIN = 11;

const int SERVO_1_PIN = 4; // D4
const int LIMIT_SWITCH_PIN = 3; //D3

// Create stepper motor objects
SpeedyStepper stepperX;
SpeedyStepper stepperY;
SpeedyStepper stepperZ;
SpeedyStepper stepperE0; // E0
SpeedyStepper stepperE1; // E1
SpeedyStepper stepperE2; // E2

Servo servo1;
//MPU6050 mpu(Wire);

// Constants for the stepper motor calculations
const float STEPS_PER_REV = 3200.0; // Typically 200 steps per revolution for a 1.8-degree stepper motor
const float LEAD_SCREW_PITCH = 2.0; // mm per revolution (example value)

// Gear ratios
const float GEAR_RATIO_XY = 27 * (36.0/(14.0*2.3)); // Reducer for X and Y
const float GEAR_RATIO_JOINT_3 = 4;
const float GEAR_RATIO_JOINT_4 = 1.0; // No gear ratio
const float GEAR_RATIO_JOINT_5 = 4;

// Function prototypes
long angleToSteps(float angle, float gearRatio);
long linearDistanceToSteps(float distance, float leadScrewPitch);
void moveJointsWithCoordination(long steps1, long steps2, long steps3, long steps4, long steps5, long steps6, float speed);
void printAnglesAndDistances(float angle1, float angle2, float angle3, float angle4, float angle5, float distance);

// Initialize the angles and distances to zero
float angle1 = 0.0;
float angle2 = 0.0;
float angle3 = 0.0;
float angle4 = 0.0;
float angle5 = 0.0;
float linearDistance = 0.0;

float current_angle1 = 0.0;
float current_angle2 = 0.0;
float current_angle3 = 0.0;
float current_angle4 = 0.0;
float current_angle5 = 0.0;
float current_linearDistance = 0.0;

// Default speed
float speed = 1500.0;
unsigned long timer = 0;

void homeJoint6() {
  const float homingSpeed = 50.0;
  const float maxHomingDistance = 800; // 38 cm maximum movement for homing
  Serial.println("Start home position for joint 6");

  // Change direction to 1 (clockwise) instead of -1 (counterclockwise)
  if (stepperE2.moveToHomeInMillimeters(1, homingSpeed, maxHomingDistance, LIMIT_SWITCH_PIN) != true) {
    // If homing fails, blink the LED fast forever indicating a problem
    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
  }

  // Homing is complete, set current position to zero
  stepperE2.setCurrentPositionInMillimeters(0.0);
  Serial.println("Home position succeeded for joint 6");
}

// Home joints 1 and 2 using MPU6050
// void homeJoints1And2() {
//   mpu.update();
//   delay(1000);
//   Serial.println("Start home position for joints 1 and 2");
//   Serial.print("Angle X: ");
//   Serial.println(mpu.getAngleX());
//   Serial.print("Angle Y: ");
//   Serial.println(mpu.getAngleY());

//   long steps1 = angleToSteps(mpu.getAngleY(), GEAR_RATIO_XY);
//   long steps2 = angleToSteps(mpu.getAngleX(), GEAR_RATIO_XY);

//   long stepsX = steps1 + steps2;
//   long stepsY = steps1 - steps2;

//   moveJointsWithCoordination(stepsX, -stepsY, 0, 0, 0, 0, speed);
//   Serial.println("Home position succeeded for joints 1 and 2");
// }
// Function to calculate relative movement
void calculateRelativeMovement(float newAngle1, float newAngle2, float newAngle3, 
                             float newAngle4, float newAngle5, float newDistance,
                             float &relAngle1, float &relAngle2, float &relAngle3,
                             float &relAngle4, float &relAngle5, float &relDistance) {
    relAngle1 = newAngle1 - current_angle1;
    relAngle2 = newAngle2 - current_angle2;
    relAngle3 = newAngle3 - current_angle3;
    relAngle4 = newAngle4 - current_angle4;
    relAngle5 = newAngle5 - current_angle5;
    relDistance = newDistance - current_linearDistance;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Enable the stepper motors
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(E0_ENABLE_PIN, OUTPUT); // E0
  pinMode(E1_ENABLE_PIN, OUTPUT); // E1
  pinMode(E2_ENABLE_PIN, OUTPUT); // E2

  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(E0_ENABLE_PIN, LOW); // E0
  digitalWrite(E1_ENABLE_PIN, LOW); // E1
  digitalWrite(E2_ENABLE_PIN, LOW); // E2

  // Attach and configure the servo motor
  servo1.attach(SERVO_1_PIN);

  // Connect and configure the stepper motors to their IO pins
  stepperX.connectToPins(MOTOR_X_STEP_PIN, MOTOR_X_DIRECTION_PIN);
  stepperY.connectToPins(MOTOR_Y_STEP_PIN, MOTOR_Y_DIRECTION_PIN);
  stepperZ.connectToPins(MOTOR_Z_STEP_PIN, MOTOR_Z_DIRECTION_PIN);
  stepperE0.connectToPins(MOTOR_E_STEP_PIN, MOTOR_E_DIRECTION_PIN); // E0
  stepperE1.connectToPins(MOTOR_E1_STEP_PIN, MOTOR_E1_DIRECTION_PIN); // E1
  stepperE2.connectToPins(MOTOR_E2_STEP_PIN, MOTOR_E2_DIRECTION_PIN); // E2

  // // Initialize MPU6050
  // byte status = mpu.begin();
  // if (status != 0) {
  //   Serial.print("MPU6050 initialization failed with code: ");
  //   Serial.println(status);
  //   while (1);
  // }
  // mpu.calcGyroOffsets(); // Calibrate gyro

  homeJoint6();
  //homeJoints1And2();
}


void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'M') { // Command to move joints
      float newAngle1 = Serial.parseFloat();
      float newAngle2 = Serial.parseFloat();
      float newAngle3 = Serial.parseFloat();
      float newAngle4 = Serial.parseFloat();
      float newAngle5 = Serial.parseFloat();
      float newLinearDistance = Serial.parseFloat();

      // Calculate relative movements
      float relAngle1, relAngle2, relAngle3, relAngle4, relAngle5, relDistance;
      calculateRelativeMovement(newAngle1, newAngle2, newAngle3, newAngle4, newAngle5, newLinearDistance,
                               relAngle1, relAngle2, relAngle3, relAngle4, relAngle5, relDistance);

      // Convert relative angles to steps
      long steps1 = angleToSteps(relAngle1, GEAR_RATIO_XY);
      long steps2 = angleToSteps(relAngle2, GEAR_RATIO_XY);
      long steps3 = angleToSteps(relAngle3, GEAR_RATIO_JOINT_3);
      long steps4 = angleToSteps(relAngle4, GEAR_RATIO_JOINT_4);
      long steps5 = angleToSteps(relAngle5, GEAR_RATIO_JOINT_5);
      long steps6 = linearDistanceToSteps(relDistance, LEAD_SCREW_PITCH);

      long stepsX = steps1 + steps2;
      long stepsY = steps1 - steps2;

      moveJointsWithCoordination(stepsX, stepsY, steps3, steps4, steps5, steps6, speed);
      
      // Update current positions after successful movement
      current_angle1 = newAngle1;
      current_angle2 = newAngle2;
      current_angle3 = newAngle3;
      current_angle4 = newAngle4;
      current_angle5 = newAngle5;
      current_linearDistance = newLinearDistance;

      Serial.print("Moved joints by: ");
      printAnglesAndDistances(relAngle1, relAngle2, relAngle3, relAngle4, relAngle5, relDistance);
      Serial.print("New position: ");
      printAnglesAndDistances(current_angle1, current_angle2, current_angle3, 
                             current_angle4, current_angle5, current_linearDistance);
    } else if (command == 'A') {
      printAnglesAndDistances(angle1, angle2, angle3, angle4, angle5, linearDistance);
    } else if (command == 'R') {
      int servo1Pos = Serial.parseInt();
      servo1.write(servo1Pos);
    } else if (command == 'S') { // Command to set speed
      speed = Serial.parseFloat();
      Serial.print("Speed set to: ");
      Serial.println(speed);
    }
  }
}

long angleToSteps(float angle, float gearRatio) {
  return (long)((angle / 360.0) * STEPS_PER_REV * gearRatio);
}

long linearDistanceToSteps(float distance, float leadScrewPitch) {
  return (long)((distance / leadScrewPitch) * STEPS_PER_REV);
}

void printAnglesAndDistances(float angle1, float angle2, float angle3, float angle4, float angle5, float distance) {
  Serial.print("Joint 1: ");
  Serial.print(angle1);
  Serial.print(" Joint 2: ");
  Serial.print(angle2);
  Serial.print(" Joint 3: ");
  Serial.print(angle3);
  Serial.print(" Joint 4: ");
  Serial.print(angle4);
  Serial.print(" Joint 5: ");
  Serial.print(angle5);
  Serial.print(" Joint 6: ");
  Serial.println(distance);
}

void moveJointsWithCoordination(long steps1, long steps2, long steps3, long steps4, long steps5, long steps6, float speed) {
  long absSteps1 = abs(steps1);
  long absSteps2 = abs(steps2);
  long absSteps3 = abs(steps3);
  long absSteps4 = abs(steps4);
  long absSteps5 = abs(steps5);
  long absSteps6 = abs(steps6);

  long maxSteps = max(max(max(max(max(absSteps1, absSteps2), absSteps3), absSteps4), absSteps5), absSteps6);

  // Calculate the speeds needed to ensure all movements complete at the same time
  float speed1 = (absSteps1 != 0) ? speed * ((float)absSteps1 / maxSteps) : 0;
  //float speed2 = (absSteps2 != 0) ? speed * ((float)absSteps2 / maxSteps) : 0;
  float speed3 = (absSteps3 != 0) ? speed * ((float)absSteps3 / maxSteps) : 0;
  float speed4 = (absSteps4 != 0) ? speed * ((float)absSteps4 / maxSteps) : 0;
  float speed5 = (absSteps5 != 0) ? speed * ((float)absSteps5 / maxSteps) : 0;
  float speed6 = (absSteps6 != 0) ? speed * ((float)absSteps6 / maxSteps) : 0;

  // Set speeds and accelerations
  stepperX.setSpeedInStepsPerSecond(speed1);
  stepperY.setSpeedInStepsPerSecond(speed1);
  stepperZ.setSpeedInStepsPerSecond(speed3);
  stepperE0.setSpeedInStepsPerSecond(speed4);
  stepperE1.setSpeedInStepsPerSecond(speed5);
  stepperE2.setSpeedInStepsPerSecond(speed6);

  stepperX.setAccelerationInStepsPerSecondPerSecond(20000);
  stepperY.setAccelerationInStepsPerSecondPerSecond(20000);
  stepperZ.setAccelerationInStepsPerSecondPerSecond(2000);
  stepperE0.setAccelerationInStepsPerSecondPerSecond(2000);
  stepperE1.setAccelerationInStepsPerSecondPerSecond(2000);
  stepperE2.setAccelerationInStepsPerSecondPerSecond(2000);

  // Setup relative moves
  stepperX.setupRelativeMoveInSteps(steps1);
  stepperY.setupRelativeMoveInSteps(steps2);
  stepperZ.setupRelativeMoveInSteps(steps3);
  stepperE0.setupRelativeMoveInSteps(steps4);
  stepperE1.setupRelativeMoveInSteps(steps5);
  stepperE2.setupRelativeMoveInSteps(steps6);

  // Process movements
  while (!stepperX.motionComplete() || !stepperY.motionComplete() || !stepperZ.motionComplete() || !stepperE0.motionComplete() || !stepperE1.motionComplete() || !stepperE2.motionComplete()) {
    stepperX.processMovement();
    stepperY.processMovement();
    stepperZ.processMovement();
    stepperE0.processMovement();
    stepperE1.processMovement();
    stepperE2.processMovement();
  }
}
