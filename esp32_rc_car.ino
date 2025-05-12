#include <Arduino.h>

// Pin definitions
// RC receiver pins
#define CH1_PIN 36  // Steering (connect to RC receiver CH1)
#define CH3_PIN 34  // Throttle (connect to RC receiver CH3)

// BTS7960 driver pins (2 drivers, one for left motors, one for right)
#define LEFT_RPWM 25  // Left motors forward PWM
#define LEFT_LPWM 26  // Left motors reverse PWM
#define RIGHT_RPWM 32  // Right motors forward PWM
#define RIGHT_LPWM 33  // Right motors reverse PWM

// RC signal variables
unsigned long ch1_start = 0;
unsigned long ch3_start = 0;
int ch1_pwm = 1500;  // steering (neutral value)
int ch3_pwm = 1500;  // throttle (neutral value)
volatile bool ch1_signal_present = false;
volatile bool ch3_signal_present = false;
unsigned long last_signal_time = 0;
const unsigned long FAILSAFE_DELAY = 500; // failsafe timeout in ms

// Motor control variables
int leftSpeed = 0;   // Range: -255 to 255
int rightSpeed = 0;  // Range: -255 to 255

// PWM setup
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
const int LEFT_MOTOR_FORWARD_CHANNEL = 0;
const int LEFT_MOTOR_REVERSE_CHANNEL = 1;
const int RIGHT_MOTOR_FORWARD_CHANNEL = 2;
const int RIGHT_MOTOR_REVERSE_CHANNEL = 3;

// RC Signal Constants
const int RC_MIN = 1000;  // minimum RC pulse width in µs
const int RC_MAX = 2000;  // maximum RC pulse width in µs
const int RC_MID = 1500;  // middle/neutral RC pulse width in µs
const int RC_DEADZONE = 50; // deadzone around mid position

// ISR for RC channel 1 (steering)
void IRAM_ATTR ch1_interrupt() {
  if (digitalRead(CH1_PIN) == HIGH) {
    ch1_start = micros();
  } else {
    ch1_pwm = micros() - ch1_start;
    ch1_signal_present = true;
    last_signal_time = millis();
  }
}

// ISR for RC channel 3 (throttle)
void IRAM_ATTR ch3_interrupt() {
  if (digitalRead(CH3_PIN) == HIGH) {
    ch3_start = micros();
  } else {
    ch3_pwm = micros() - ch3_start;
    ch3_signal_present = true;
    last_signal_time = millis();
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configure RC receiver pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  
  // Attach interrupts for RC signals
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), ch3_interrupt, CHANGE);

  // Configure PWM for motor drivers
  ledcSetup(LEFT_MOTOR_FORWARD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LEFT_MOTOR_REVERSE_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_FORWARD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_REVERSE_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  
  // Attach PWM channels to GPIO pins
  ledcAttachPin(LEFT_RPWM, LEFT_MOTOR_FORWARD_CHANNEL);
  ledcAttachPin(LEFT_LPWM, LEFT_MOTOR_REVERSE_CHANNEL);
  ledcAttachPin(RIGHT_RPWM, RIGHT_MOTOR_FORWARD_CHANNEL);
  ledcAttachPin(RIGHT_LPWM, RIGHT_MOTOR_REVERSE_CHANNEL);
  
  Serial.println("ESP32 RC Car System Initialized");
}

// Map RC value to motor speed with deadzone
int mapWithDeadzone(int value, int inMin, int inMid, int inMax, int outMin, int outMax) {
  // Apply deadzone around center
  if (abs(value - inMid) < RC_DEADZONE) {
    return 0;
  }
  
  if (value < inMid) {
    return map(value, inMin, inMid - RC_DEADZONE, outMin, 0);
  } else {
    return map(value, inMid + RC_DEADZONE, inMax, 0, outMax);
  }
}

// Set motor speed and direction
void setMotorSpeed(int leftMotorSpeed, int rightMotorSpeed) {
  // Constrain values
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
  
  // Set left motors
  if (leftMotorSpeed >= 0) {
    ledcWrite(LEFT_MOTOR_FORWARD_CHANNEL, leftMotorSpeed);
    ledcWrite(LEFT_MOTOR_REVERSE_CHANNEL, 0);
  } else {
    ledcWrite(LEFT_MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(LEFT_MOTOR_REVERSE_CHANNEL, -leftMotorSpeed);
  }
  
  // Set right motors
  if (rightMotorSpeed >= 0) {
    ledcWrite(RIGHT_MOTOR_FORWARD_CHANNEL, rightMotorSpeed);
    ledcWrite(RIGHT_MOTOR_REVERSE_CHANNEL, 0);
  } else {
    ledcWrite(RIGHT_MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(RIGHT_MOTOR_REVERSE_CHANNEL, -rightMotorSpeed);
  }
}

void loop() {
  // Validate RC signals
  ch1_pwm = constrain(ch1_pwm, RC_MIN, RC_MAX);
  ch3_pwm = constrain(ch3_pwm, RC_MIN, RC_MAX);
  
  // Check if signal is present (failsafe)
  if (millis() - last_signal_time > FAILSAFE_DELAY) {
    // Signal lost - activate failsafe
    ch1_signal_present = false;
    ch3_signal_present = false;
    setMotorSpeed(0, 0);
    Serial.println("Failsafe activated - stopping motors");
    delay(100);
    return;
  }
  
  // Calculate base throttle (-255 to 255)
  int throttle = mapWithDeadzone(ch3_pwm, RC_MIN, RC_MID, RC_MAX, -255, 255);
  
  // Calculate steering (-100% to 100%)
  float steering = map(ch1_pwm, RC_MIN, RC_MAX, -100, 100) / 100.0;
  
  // Apply differential steering logic
  if (throttle > 0) { // Moving forward
    if (steering > 0) { // Turning right
      leftSpeed = throttle;
      rightSpeed = throttle * (1 - abs(steering));
    } else { // Turning left
      leftSpeed = throttle * (1 - abs(steering));
      rightSpeed = throttle;
    }
  } else if (throttle < 0) { // Moving in reverse
    if (steering > 0) { // Turning right
      leftSpeed = throttle;
      rightSpeed = throttle * (1 - abs(steering));
    } else { // Turning left
      leftSpeed = throttle * (1 - abs(steering));
      rightSpeed = throttle;
    }
  } else { // No throttle input
    // Spin in place if only steering is provided
    if (abs(steering) > 0.2) {
      leftSpeed = 128 * steering;
      rightSpeed = -128 * steering;
    } else {
      leftSpeed = 0;
      rightSpeed = 0;
    }
  }
  
  // Apply motor speeds
  setMotorSpeed(leftSpeed, rightSpeed);
  
  // Debug output (every 200ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("CH1 (Steering): ");
    Serial.print(ch1_pwm);
    Serial.print(" | CH3 (Throttle): ");
    Serial.print(ch3_pwm);
    Serial.print(" | Left Motors: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Motors: ");
    Serial.println(rightSpeed);
    
    lastPrint = millis();
  }
  
  delay(10);
}
