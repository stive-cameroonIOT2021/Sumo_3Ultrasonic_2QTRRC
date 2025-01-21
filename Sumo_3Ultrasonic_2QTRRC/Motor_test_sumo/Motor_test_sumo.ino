/*
  TB6612FNG H-Bridge Demo
  TB6612-Demo.ino
  Demonstrates use of TB6612FNG H-Bridge Motor Controller
  Drives two DC Motors

  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
// Define motor control pins for Port D
#define LEFT_MOTOR_IN1  PD3   // Motor Left IN1
#define LEFT_MOTOR_IN2  PD4   // Motor Left IN2
#define RIGHT_MOTOR_IN1 PD5   // Motor Right IN1
#define RIGHT_MOTOR_IN2 PD6   // Motor Right IN2

// PWM pins for speed control
#define LEFT_PWM_PIN  9
#define RIGHT_PWM_PIN 10

#define MAX_SPPED 200

// Reverse motor flags
bool is_reverse_left = false;  // Set true if left motor needs to be reversed
bool is_reverse_right = false; // Set true if right motor needs to be reversed

// Speed settings for the motors
int baseSpeed = 100;  // Base speed

void setup(){
  // Initialize motor pins and PWM
  setupPWM();
   
}

void loop() {
  
  // Adjust motor speeds using the correction value
  int leftSpeed = 200;
  int rightSpeed = 200;

  // Constrain motor speeds to avoid exceeding the maximum PWM value (255)
  leftSpeed = constrain(leftSpeed, -MAX_SPPED, MAX_SPPED);
  rightSpeed = constrain(rightSpeed, -MAX_SPPED, MAX_SPPED);

  // Move the motors based on calculated speeds
  move(leftSpeed, rightSpeed);
  /* delay(4000);
  move(-leftSpeed, -rightSpeed);;
  delay(4000); */
  
}

// Function to set up DDR and timers for PWM
void setupPWM() {

  // Set direction of IN1 and IN2 pins to output (DDRD for Port D)
  DDRD |= (1 << LEFT_MOTOR_IN1) | (1 << LEFT_MOTOR_IN2) | (1 << RIGHT_MOTOR_IN1) | (1 << RIGHT_MOTOR_IN2);
  
  // Set up Timer1 for PWM (Fast PWM mode, non-inverting)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  // Fast PWM, non-inverting mode
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);     // Fast PWM, Prescaler 8

  ICR1 = 255;  // Set TOP value for 8-bit resolution
}

// Function to move both motors with independent speeds using analogWrite
void move(int leftSpeed, int rightSpeed) {
  // Adjust left motor direction based on is_reverse_left flag
  if (is_reverse_left) {
    leftSpeed = -leftSpeed;  // Reverse the left motor speed
  }

  // Adjust right motor direction based on is_reverse_right flag
  if (is_reverse_right) {
    rightSpeed = -rightSpeed;  // Reverse the right motor speed
  }

  // Handle left motor direction
  if (leftSpeed == 0) {
    PORTD &= ~(1 << LEFT_MOTOR_IN1);
    PORTD &= ~(1 << LEFT_MOTOR_IN2);
    analogWrite(LEFT_PWM_PIN, 0);  // Stop PWM for left motor
  } else if (leftSpeed < 0) {
    PORTD &= ~(1 << LEFT_MOTOR_IN1);  // Set IN1 LOW
    PORTD |= (1 << LEFT_MOTOR_IN2);   // Set IN2 HIGH for reverse
    analogWrite(LEFT_PWM_PIN, abs(leftSpeed));  // Control speed via PWM
  } else {
    PORTD |= (1 << LEFT_MOTOR_IN1);   // Set IN1 HIGH for forward
    PORTD &= ~(1 << LEFT_MOTOR_IN2);  // Set IN2 LOW
    analogWrite(LEFT_PWM_PIN, leftSpeed);  // Control speed via PWM
  }

  // Handle right motor direction
  if (rightSpeed == 0) {
    PORTD &= ~(1 << RIGHT_MOTOR_IN1);
    PORTD &= ~(1 << RIGHT_MOTOR_IN2);
    analogWrite(RIGHT_PWM_PIN, 0);  // Stop PWM for right motor
  } else if (rightSpeed < 0) {
    PORTD &= ~(1 << RIGHT_MOTOR_IN1); // Set IN1 LOW
    PORTD |= (1 << RIGHT_MOTOR_IN2);  // Set IN2 HIGH for reverse
    analogWrite(RIGHT_PWM_PIN, abs(rightSpeed));  // Control speed via PWM
  } else {
    PORTD |= (1 << RIGHT_MOTOR_IN1);  // Set IN1 HIGH for forward
    PORTD &= ~(1 << RIGHT_MOTOR_IN2); // Set IN2 LOW
    analogWrite(RIGHT_PWM_PIN, rightSpeed);  // Control speed via PWM
  }
}