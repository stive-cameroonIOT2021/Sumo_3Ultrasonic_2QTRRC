#include <QTRSensors.h>
#include <Ultrasonic.h>

/* ------------------------------------------------------------------------------------- */

#define SONAR_NUM     3 // Number of sensors.

Ultrasonic ultrasonic1(A0, A1);    // Left(trig, echo)
Ultrasonic ultrasonic2(A2, A3);    // Front(trig, echo)
Ultrasonic ultrasonic3(11, 12);    // Right(trig, echo)

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
/* -------------------------------Motors setup------------------------------------------------------ */
// Define motor control pins for Port D
#define LEFT_MOTOR_IN1  PD3   // Motor Left IN1
#define LEFT_MOTOR_IN2  PD4   // Motor Left IN2
#define RIGHT_MOTOR_IN1 PD5   // Motor Right IN1
#define RIGHT_MOTOR_IN2 PD6   // Motor Right IN2

// PWM pins for speed control
#define LEFT_PWM_PIN  9
#define RIGHT_PWM_PIN 10

// Reverse motor flags
bool is_reverse_left = false;  // Set true if left motor needs to be reversed
bool is_reverse_right = false; // Set true if right motor needs to be reversed

/* ------------------------------------------------------------------------------------- */

QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
/* ------------------------------------------------------------------------------------- */
const int startBtn = 2,
          ArduLed = 13;

/* ------------------------------------Action state robot------------------------------------------------- */
enum ActionState { WAITING, ATTACK, TURN_LEFT, TURN_RIGHT, SEARCH_TARGET };

ActionState currentActionState = WAITING;
/* ------------------------------------------------------------------------------------- */
// Constants
const int edgeThreshold = 700, 
          MaxSpeed = 255, 
          normalSpeed = 220, 
          TurnSpeed = 200, 
          edgeAvoidanceSpeed = 200;

const unsigned long edgeAvoidanceDuration = 200, 
                    debounceTime = 50, 
                    actionInterval = 100;

//Variables
bool robotStarted = false, avoidingEdge = false;
unsigned long lastEdgeMillis = 0, 
              lastButtonPressTime = 0, 
              lastActionTime = 0;

const int T_before_start = 5000;//Time robot start after button pressed

const int Dist_Treshold = 18;// Distance to recognize opponent


void setup() {
  // Initialize motor pins and PWM
  setupPWM();
  move(0,0);//Stop motors 

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 8}, SensorCount);//Left, Right
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(startBtn, INPUT);

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);
}

void loop()
{

  if (!robotStarted) {
      if (digitalRead(startBtn) == HIGH && (millis() - lastButtonPressTime >= debounceTime)) {
          delay(T_before_start);
          lastButtonPressTime = millis();
          robotStarted = true;
          currentActionState = SEARCH_TARGET;
      } else digitalWrite(ArduLed, (millis() / 500) % 2 == 0);
  } else {
    //move(140, 200);
    qtr.read(sensorValues);
  /* for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
Serial.println(); */
    ReadDistances_MakeDecision();
    //currentActionState = SEARCH_TARGET;
    performAction();
    //delay(250);
    
  }

}

/* --------------------------------FUNCTIONS------------------------------------------------ */
void searchTarget() {
    if (avoidingEdge) {
        if (millis() - lastEdgeMillis < edgeAvoidanceDuration) {
            sensorValues[0] < edgeThreshold ? move(edgeAvoidanceSpeed - 60, -edgeAvoidanceSpeed) : move(-edgeAvoidanceSpeed + 60, edgeAvoidanceSpeed);
        } else avoidingEdge = false;
    } else {
        if (sensorValues[0] < edgeThreshold || sensorValues[1] < edgeThreshold) {
            move(-edgeAvoidanceSpeed + 60, -edgeAvoidanceSpeed);//move backward
            lastEdgeMillis = millis();
            avoidingEdge = true;
        } else move(normalSpeed - 60, normalSpeed); //since no edge detect, move freely
    }
}

void performAction() {
    if (millis() - lastActionTime < actionInterval) return;
    lastActionTime = millis();
    
    switch (currentActionState) {
        case ATTACK:       move(MaxSpeed - 60, MaxSpeed); break;
        case TURN_LEFT:    move(-TurnSpeed + 60, TurnSpeed); break;
        case TURN_RIGHT:   move(TurnSpeed - 60, -TurnSpeed); break;
        case SEARCH_TARGET: searchTarget(); break;
        case WAITING:      move(0, 0); break;
    }
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

void ReadDistances_MakeDecision() {
    // Store the distances in the global array
    cm[0] = ultrasonic1.read();  // Left sensor
    cm[1] = ultrasonic2.read();  // Front sensor
    cm[2] = ultrasonic3.read();  // Right sensor

    if ((cm[1] < Dist_Treshold) && sensorValues[0] > edgeThreshold && sensorValues[1] > edgeThreshold) currentActionState = ATTACK;
    else if((cm[0] < Dist_Treshold) && sensorValues[0] > edgeThreshold && sensorValues[1] > edgeThreshold) currentActionState = TURN_LEFT;
    else if ((cm[2] < Dist_Treshold) && sensorValues[0] > edgeThreshold && sensorValues[1] > edgeThreshold) currentActionState = TURN_RIGHT;
    else currentActionState = SEARCH_TARGET;

// Debug print the sensor distances
    /* for (uint8_t i = 0; i < SONAR_NUM; i++) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(cm[i]);
      Serial.println(" cm"); 
    } 
    Serial.println(); */
}


