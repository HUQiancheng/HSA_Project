#include <Arduino.h>
#include "motor_control.h"

const int ENA = 5, IN1 = 6, IN2 = 7;
const int ENB = 9, IN3 = 10, IN4 = 11;

void setupMotors()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
}

void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotor(int pwmA, int dirA, int pwmB, int dirB)
{
  if (pwmA == 0)
  {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(ENA, pwmA);
    digitalWrite(IN1, dirA == 1);
    digitalWrite(IN2, dirA == 0);
  }
  if (pwmB == 0)
  {
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENB, pwmB);
    digitalWrite(IN3, dirB == 1);
    digitalWrite(IN4, dirB == 0);
  }
}