/*
thông số chân:
 in1 25, in2 26, pwma 27, en1a 5, en1b 17, (trái)      
 in3 32, in4 33, pwmb 14, en2a19, en2b 18 (phải)


kp=0.06

R xe = 1,5cm
xung 210 
tỉ số truyền 1:30




*/
#include <Wire.h>
// Encoder pins
#define ENCA1 5  // Encoder A (Left motor)
#define ENCB1 17 // Encoder B (Left motor)
#define ENCA2 19 // Encoder A (Right motor)
#define ENCB2 18 // Encoder B (Right motor)

// Motor A (Left)
#define PWM_A 27
#define IN1_A 25
#define IN2_A 26

// Motor B (Right)
#define PWM_B 14
#define IN3_B 32
#define IN4_B 33

volatile long posLeft = 0;
volatile long posRight = 0;

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eprev = 0;
float eintegral = 0;
int errorx = 1;
int errory = 30;
int errorz = 10;
#define TOF_FRONT_ADDRESS 0x46  // Front sensor I2C address
#define TOF_LEFT_ADDRESS  0x4B  // Left sensor I2C address
#define TOF_RIGHT_ADDRESS 0x52  // Right sensor I2C address

// Define sensor registers (based on the TOF10120 sensor datasheet)
#define TOF_REG_DISTANCE 0x00  // Register to read distance data

#define NUM_READINGS 10  // Number of readings to average
uint16_t frontDistanceReadings[NUM_READINGS];
uint16_t leftDistanceReadings[NUM_READINGS];
uint16_t rightDistanceReadings[NUM_READINGS];
#define DESIRED_WALL_DISTANCE 40  // Desired distance from the wall (in mm)
#define INPUT_DISTANCE 150  // in mm (converted from cm to match TOF10120 units)
#define ERROR_DIST 50  // in mm (converted to mm)
#define SPEED 95
#define MAX_DIFFERNCE 10
#define MAX_TURN_SPEED 10
#define CALIBRATION 3
#define MAX_ALLIGN_ANGLE 5  // in mm
#define COLLISION_DISTANCE 100  // in mm

// Constants for calibration
const float Kp = 1.8;   // Proportional constant to fix straight movement
const float Kd = 0;  // Derivative constant for adjusting drift
const float Ki = 0;   // Integral constant

const int targetTicksStraight = 450;  // Adjusted ticks for forward movement
const int targetTicks = 200;  //  Adjusted encoder ticks for 90-degree turn 
const int speed = 200; //1-255
void setup() {
  Serial.begin(9600);
  
  // Initialize encoder pins
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoderRight, RISING);
  
  // Initialize motor pins
  pinMode(PWM_A, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  
  pinMode(PWM_B, OUTPUT);
  pinMode(IN3_B, OUTPUT);
  pinMode(IN4_B, OUTPUT);
}

void loop() {
  // In giá trị encoder ra màn hình Serial mỗi giây
  uint16_t frontDistance = readTOFSensor(TOF_FRONT_ADDRESS);
  uint16_t leftDistance = readTOFSensor(TOF_LEFT_ADDRESS);
  uint16_t rightDistance = readTOFSensor(TOF_RIGHT_ADDRESS);
  Serial.print("Encoder Left: ");
  Serial.print(posLeft);
  Serial.print("  Encoder Right: ");
  Serial.println(posRight);
  // Stop after turning 90 degrees
  // Perform 90-degree right turn
  //moveForward1();
  //delay(500);
  // // Move forward after turning
  turnRight();

  // delay(500);// Stop the robot after moving forward
  // delay(500);
  //turnAround();
  // delay(500);
  // turnaround();
  // stopMotors();
  // delay(5000);  // Pause for 5 seconds before restarting the loop
  // if (check_collision(frontDistance)) {
  //   delay(500);
  //   turnRight();
  //   delay(500);
  // }
  delay(3000);
}

// Function to read encoder value for the left motor
void readEncoderLeft() {
  if (digitalRead(ENCB1) > 0) {
    posLeft++;
  } else {
    posLeft--;
  }
}

// Function to read encoder value for the right motor
void readEncoderRight() {
  if (digitalRead(ENCB2) > 0) {
    posRight++;
  } else {
    posRight--;
  }
}

// Function to control motor direction and speed
void setMotor(float control, int pwmPin, int in1, int in2) {
  int pwr = abs(control);
  if (pwr > 255) pwr = 255;

  analogWrite(pwmPin, pwr);
  
  if (control > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (control < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
// Function to stop both motors
void stopMotors() {
  setMotor(0, PWM_A, IN1_A, IN2_A);
  setMotor(0, PWM_B, IN3_B, IN4_B);
}

//Function to move forward while keeping the robot straight
void moveForward() {
  posLeft = 0;
  posRight = 0;
  int pos = 0;
  // while (abs(pos) < targetTicksStraight) {
  //   //Left motor forward, right motor backward for turning
  //   noInterrupts();
  //   pos = (abs(posLeft) + abs(posRight)) / 2;
  //   interrupts();
  //   int error = targetTicksStraight - abs(pos);
  //   float PID = 1 * error;
  //   setMotor(PID, PWM_A, IN1_A, IN2_A);   // Adjust speed as necessary
  //   setMotor(-PID, PWM_B, IN3_B, IN4_B);  // Adjust speed as necessary
  //   Serial.println(posRight);
  //   delay(1);
  // }
  while (abs(posRight) < targetTicksStraight && abs(posLeft) < targetTicksStraight) {
  //Left motor forward, right motor backward for turning
    setMotor(speed, PWM_A, IN1_A, IN2_A);   // Adjust speed as necessary
    setMotor(-speed, PWM_B, IN3_B, IN4_B);  // Adjust speed as necessary
    delay(1);
  }
  stopMotors();  // Stop after turning 90 degrees
}
// Function to perform a 90-degree left turn
void turnLeft(){
  posLeft = 0;
  posRight = 0;
  int targetTicksLeft = targetTicks - errorx - errory - errorz;
  int targetTicksRight = targetTicks + errory;
  // Continue moving forward while checking encoders
  while (abs(posLeft) < targetTicksLeft && abs(posRight) < targetTicksRight) {
    // Calculate the difference in encoder counts between left and right motors
    int error = posLeft - posRight;
    
    // Adjust motor speeds to minimize the drift
    float correction = Kp * error + Kd * (eprev - error);
    eintegral += error;
    eprev = error;
    setMotor(-speed - correction, PWM_A, IN1_A, IN2_A);  // Adjust left motor speed
    setMotor(-speed - correction, PWM_B, IN3_B, IN4_B);  // Adjust right motor speed
    delay(5);  // Tốc độ vòng lặp
  }
  stopMotors();  // Stop after moving forward the set distance

  stopMotors();  // Stop after moving forward the set distance
}
// Function to perform a 90-degree right turn
void turnRight() {
  posLeft = 0;
  posRight = 0;
  int targetTicksLeft = targetTicks - errorx + errory;
  int targetTicksRight = targetTicks + errory;
  // Continue moving forward while checking encoders
  while (abs(posLeft) < targetTicksLeft && abs(posRight) < targetTicksRight) {
    // Calculate the difference in encoder counts between left and right motors
    int error = posLeft - posRight;
    
    // Adjust motor speeds to minimize the drift
    float correction = Kp * error + Kd * (eprev - error);
    eintegral += error;
    eprev = error;
    setMotor(speed - correction, PWM_A, IN1_A, IN2_A);  // Adjust left motor speed
    setMotor(speed + correction, PWM_B, IN3_B, IN4_B);  // Adjust right motor speed
    delay(5);  // Tốc độ vòng lặp
  }
  stopMotors();  // Stop after moving forward the set distance
}
void turnAround(){
  for(int i = 0; i < 2; i++){
    turnRight();
  }
}
// Collision detection using the front TOF sensor
bool check_collision(uint16_t Distance) {
  return Distance < COLLISION_DISTANCE;
}

// Function to read distance from a TOF sensor
uint16_t readTOFSensor(uint8_t sensorAddress) {
  uint16_t distance = 0;
  Wire.beginTransmission(sensorAddress);
  Wire.write(TOF_REG_DISTANCE);  // Request distance register
  Wire.endTransmission();

  Wire.requestFrom(sensorAddress, (uint8_t)2);  // Request 2 bytes
  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    distance = (highByte << 8) | lowByte;  // Combine into 16-bit value
  }
  return distance;
}
// Function to follow the wall using the left TOF sensor
void followWall() {
  posLeft = 0;
  posRight = 0;
  int pos = 0;
  
  uint16_t leftDistance = readTOFSensor(TOF_LEFT_ADDRESS);  // Read the left TOF sensor

  while (true) {  // Continuous loop to keep following the wall
    // Read the current distance from the wall
    leftDistance = readTOFSensor(TOF_LEFT_ADDRESS);
    
    // Calculate error (difference between desired distance and current distance)
    int error = DESIRED_WALL_DISTANCE - leftDistance;
    
    // Apply PID control to calculate correction
    float correction = Kp * error + Kd * (eprev - error) + Ki * eintegral;
    eintegral += error;
    eprev = error;
    
    // Adjust motor speeds based on correction
    int motorSpeedLeft = speed - correction;  // Adjust left motor
    int motorSpeedRight = speed + correction;  // Adjust right motor
    
    // Make sure the motor speeds stay within valid bounds
    if (motorSpeedLeft > 255) motorSpeedLeft = 255;
    if (motorSpeedLeft < -255) motorSpeedLeft = -255;
    if (motorSpeedRight > 255) motorSpeedRight = 255;
    if (motorSpeedRight < -255) motorSpeedRight = -255;
    
    // Set the motor speeds
    setMotor(motorSpeedLeft, PWM_A, IN1_A, IN2_A);   // Left motor control
    setMotor(motorSpeedRight, PWM_B, IN3_B, IN4_B);  // Right motor control

    // Break loop if collision is detected (optional)
    uint16_t frontDistance = readTOFSensor(TOF_FRONT_ADDRESS);
    if (check_collision(frontDistance)) {
      stopMotors();  // Stop the robot if a collision is detected
      break;
    }

    delay(10);  // Small delay for smoother movement
  }
}
void readVelocity(){
  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 100*(sin(currT/1e6)>0);

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;
};
void moveForward1() {
  posLeft = 0;
  posRight = 0;
  int pos = 0;
  int targetTicksLeft = targetTicksStraight;
  int targetTicksRight = targetTicksStraight + errorx;
  // Continue moving forward while checking encoders
  while (abs(posLeft) < targetTicksLeft && abs(posRight) < targetTicksRight) {
    noInterrupts(); // disable interrupts temporarily while reading
    int pos = (abs(posLeft) + abs(posRight))/2;
    interrupts(); // turn interrupts back on
    // Calculate the error between the two encoder readings
    int error = posRight + posLeft;
    int errory = targetTicksStraight - pos;
    // Apply a simple proportional controller to correct the drift
    float correction = Kp * error;
    // Set the motor speeds
    setMotor(speed - correction, PWM_A, IN1_A, IN2_A);   // Left motor forward
    setMotor(-speed - correction, PWM_B, IN3_B, IN4_B);  // Right motor forward

    delay(10);  // Small delay for smoother movement
  }

  stopMotors();  // Stop after reaching the target distance
}
