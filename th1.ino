/*
xung 210 
tỉ số truyền 1:30
thông số chân:
 in1 25, in2 26, pwma 27, en1a 5, en1b 17, (trái)      
 in3 32, in4 33, pwmb 14, en2a19, en2b 18 (phải)









*/
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

long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Constants for calibration
const float Kp = 0.1;   // Proportional constant to fix straight movement
const float Kd = 0.02;  // Derivative constant for adjusting drift
const float Ki = 0.02;   // Integral constant
const int targetTicksStraight = 180;  // Adjusted encoder ticks for 90-degree turn


const int targetTicksTurn = 125;  // Adjusted ticks for forward movement
const int targetforward = 210;
// Function to show data on Serial Plotter
void showOnSerialPlotter(int error, float correction, int motorLeft, int motorRight) {
  // Print in a format the Serial Plotter can understand
  Serial.print("posLeft: ");
  Serial.print(posLeft);
  Serial.print(", posRight: ");
  Serial.print(posRight);
  Serial.print(", error: ");
  Serial.print(error);
  Serial.print(", correction: ");
  Serial.print(correction);
  Serial.print(", motorLeft: ");
  Serial.print(motorLeft);
  Serial.print(", motorRight: ");
  Serial.println(motorRight);  // Newline at the end
}

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
  
  Serial.print("Encoder Left: ");
  Serial.print(posLeft);
  Serial.print("  Encoder Right: ");
  Serial.println(posRight);
  // Stop after turning 90 degrees
  // Perform 90-degree right turn
  moveforward();
  delay(500);
  // Move forward after turning
  turnright90(targetTicksTurn);  // Move forward for 500 encoder ticks (adjustable)

  delay(500);// Stop the robot after moving forward
  moveforward();
  delay(500);
  turnLeft90(targetTicksTurn);

  //stopMotors();
  //delay(5000);  // Pause for 5 seconds before restarting the loop
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
void moveforward() {
  posLeft = 0;
  posRight = 0;
  int pos;
  //while (abs(pos) < targetTicksStraight) {
    // Left motor forward, right motor backward for turning
  //  noInterrupts();
  //  pos = (posLeft + posRight) / 2;
  //  interrupts();
  //  int error = targetTicksStraight - abs(pos);
  //  float PID = 1 * error;
  //  setMotor(PID, PWM_A, IN1_A, IN2_A);   // Adjust speed as necessary
  //  setMotor(- PID * 0.1, PWM_B, IN3_B, IN4_B);  // Adjust speed as necessary
  //  Serial.println(posRight);
  //  delay(1);
  //}
  while (abs(posRight) < targetTicksTurn) {
    // Left motor forward, right motor backward for turning
    setMotor(150, PWM_A, IN1_A, IN2_A);   // Adjust speed as necessary
    setMotor(-150, PWM_B, IN3_B, IN4_B);  // Adjust speed as necessary
  }
  delay(1);
  stopMotors();  // Stop after turning 90 degrees
}
// Function to perform a 90-degree left turn
void turnLeft90(int targetTicks){
  posLeft = 0;
  posRight = 0;

  while (abs(posLeft) < targetTicks && abs(posRight) < targetTicks) {
    
    // Calculate the difference in encoder counts between left and right motors
    int error = posRight - posLeft;
    
    // Adjust motor speeds to minimize the drift
    float correction = Kp * error + Kd * (eprev - error);
    eprev = error;
    
    setMotor(255 - correction, PWM_A, IN2_A, IN1_A);  // Adjust left motor speed
    setMotor(255 + correction, PWM_B, IN4_B, IN3_B);  // Adjust right motor speed
  }

  stopMotors();  // Stop after moving forward the set distance
}
// Function to perform a 90-degree right turn
void turnright90(long targetTicks) {
  posLeft = 0;
  posRight = 0;

  while (abs(posLeft) < targetTicks && abs(posRight) < targetTicks) {
    // Calculate the difference in encoder counts between left and right motors
    int error = posLeft - posRight;
    
    // Adjust motor speeds to minimize the drift
    float correction = Kp * error + Kd * (eprev - error);
    eprev = error;
    setMotor(255 - correction, PWM_A, IN1_A, IN2_A);  // Adjust left motor speed
    setMotor(255 + correction, PWM_B, IN3_B, IN4_B);  // Adjust right motor speed
    showOnSerialPlotter(error, correction, posLeft, posRight);
  }
  stopMotors();  // Stop after moving forward the set distance
}