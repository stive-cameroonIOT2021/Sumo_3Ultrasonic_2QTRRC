#include <QTRSensors.h>
#include <Ultrasonic.h>

/* --- Definitions and Constants --- */
#define SONAR_NUM 3  // Number of ultrasonic sensors
#define LEFT_MOTOR_IN1 PD3
#define LEFT_MOTOR_IN2 PD4
#define RIGHT_MOTOR_IN1 PD5
#define RIGHT_MOTOR_IN2 PD6
#define LEFT_PWM_PIN 9
#define RIGHT_PWM_PIN 10

#define EDGE_THRESHOLD 700
#define DIST_THRESHOLD 18
#define T_BEFORE_START 2000
#define DEBOUNCE_TIME 50

const int MAX_SPEED = 100;
const int TURN_SPEED = 100;
const int EDGE_TURN = 65;
const int DURATION = 100;

bool is_reverse_left = false;   // Reverse flag for left motor
bool is_reverse_right = false;  // Reverse flag for right motor

/* --- Variables --- */
unsigned int cm[SONAR_NUM];  // Stores distances from ultrasonic sensors
QTRSensors qtr;
const uint8_t sensorCount = 2;
uint16_t sensorValues[sensorCount];

unsigned long lastButtonPressTime = 0;
bool robotStarted = false;
int lastAction = 5;  // Tracks the last movement (e.g., forward)

/* --- Ultrasonic Sensors --- */
Ultrasonic ultrasonic1(A0, A1);  // Left (trig, echo)
Ultrasonic ultrasonic2(A2, A3);  // Front (trig, echo)
Ultrasonic ultrasonic3(11, 12);  // Right (trig, echo)

/* --- Setup --- */
void setup() {
  setupPWM();
  move(0, 0, 1);  // Stop motors

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 8}, sensorCount);  // Left, Right
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, INPUT);  // Start button

  Serial.begin(9600);
}

/* --- Main Loop --- */
void loop() {
  if (!robotStarted) {
    handleStartup();
  } else {
    qtr.read(sensorValues);
    readDistances();

    if (isEdgeDetected()) {
      handleEdgeDetection();
    } else {
      handleMovement();
    }
  }
}

/* --- Helper Functions --- */
void handleStartup() {
  if (digitalRead(2) == HIGH && (millis() - lastButtonPressTime >= DEBOUNCE_TIME)) {
    delay(T_BEFORE_START);
    lastButtonPressTime = millis();
    robotStarted = true;
  } else {
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2 == 0);
  }
}

void readDistances() {
  cm[0] = ultrasonic1.read();  // Left sensor
  cm[1] = ultrasonic2.read();  // Front sensor
  cm[2] = ultrasonic3.read();  // Right sensor;
}

bool isEdgeDetected() {
  return sensorValues[0] < EDGE_THRESHOLD || sensorValues[1] < EDGE_THRESHOLD;
}

void handleEdgeDetection() {
  digitalWrite(LED_BUILTIN, HIGH);  // Indicate edge detection

  if (sensorValues[0] < EDGE_THRESHOLD && sensorValues[1] >= EDGE_THRESHOLD) {
    // Left edge detected
    move(-100, -100, 200);  // Move back
    move(TURN_SPEED, -40, 200);  // Turn right
  } else if (sensorValues[1] < EDGE_THRESHOLD && sensorValues[0] >= EDGE_THRESHOLD) {
    // Right edge detected
    move(-100, -100, 200);  // Move back
    move(-40, TURN_SPEED, 200);  // Turn left
  } else if (sensorValues[0] < EDGE_THRESHOLD && sensorValues[1] < EDGE_THRESHOLD) {
    // Both edges detected
    move(-100, -100, 300);  // Move back
    move(-TURN_SPEED, TURN_SPEED, 400);  // Half-spin
  }

  lastAction = 5;  // Reset last action to forward
}

void handleMovement() {
  if (cm[1] < DIST_THRESHOLD) {
    move(MAX_SPEED, MAX_SPEED, 1);  // Move forward
    lastAction = 5;
  } else if (cm[0] < DIST_THRESHOLD) {
    move(-100, -100, 200);  // Move back
    move(-40, TURN_SPEED, 200);  // Turn left
    lastAction = 7;
  } else if (cm[2] < DIST_THRESHOLD) {
    move(-100, -100, 200);  // Move back
    move(TURN_SPEED, -40, 200);  // Turn right
    lastAction = 3;
  } else {
    executeDefaultAction();
  }
}


void executeDefaultAction() {
  switch (lastAction) {
    case 5:
      move(MAX_SPEED - 10, MAX_SPEED - 10, 1);  // Forward
      break;
    case 7:
      move(-20, TURN_SPEED, 2);  // Turn left
      break;
    case 3:
      move(TURN_SPEED, -20, 2);  // Turn right
      break;
  }
}

void setupPWM() {
  DDRD |= (1 << LEFT_MOTOR_IN1) | (1 << LEFT_MOTOR_IN2) | (1 << RIGHT_MOTOR_IN1) | (1 << RIGHT_MOTOR_IN2);
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 255;
}

void move(int leftSpeed, int rightSpeed, int timex) {
  leftSpeed = constrain(leftSpeed * 2.5, -255, 255);
  rightSpeed = constrain(rightSpeed * 2.5, -255, 255);

  if (is_reverse_left) leftSpeed = -leftSpeed;
  if (is_reverse_right) rightSpeed = -rightSpeed;

  controlMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_PWM_PIN, leftSpeed);
  controlMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_PWM_PIN, rightSpeed);

  if (timex > 0) delay(timex);
}

void controlMotor(int in1, int in2, int pwmPin, int speed) {
  if (speed == 0) {
    PORTD &= ~(1 << in1) & ~(1 << in2);
    analogWrite(pwmPin, 0);
  } else if (speed > 0) {
    PORTD |= (1 << in1);
    PORTD &= ~(1 << in2);
    analogWrite(pwmPin, speed);
  } else {
    PORTD &= ~(1 << in1);
    PORTD |= (1 << in2);
    analogWrite(pwmPin, abs(speed));
  }
}
