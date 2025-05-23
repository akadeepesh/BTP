#include <VirtualWire.h>

// Motor Pins
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;
//const int ENA = 6;  // PWM
//const int ENB = 9;  // PWM

const int rfRxPin = 11;
const int commLed = 7;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //pinMode(ENA, OUTPUT);
  //pinMode(ENB, OUTPUT);
  pinMode(commLed, OUTPUT);

  vw_set_rx_pin(rfRxPin);
  vw_setup(2000);
  vw_rx_start();

  stopMotors();
}

void loop() {
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  if (vw_get_message(buf, &buflen)) {
    char command = (char)buf[0];

    digitalWrite(commLed, HIGH); // Blink LED on receive

    switch (command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopMotors(); break;
    }

    delay(50);
    digitalWrite(commLed, LOW);
  }
}

// Motor Functions
void moveForward() {
  //analogWrite(ENA, 180);
  //analogWrite(ENB, 180);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  //analogWrite(ENA, 180);
  //analogWrite(ENB, 180);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  //analogWrite(ENA, 150);
  //analogWrite(ENB, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  //analogWrite(ENA, 150);
  //analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  //analogWrite(ENA, 0);
//  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
