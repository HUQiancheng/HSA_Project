#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void setupMotors();
void stopMotors();
void setMotor(int pwmA, int dirA, int pwmB, int dirB);

#endif