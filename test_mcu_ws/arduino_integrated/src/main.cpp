#include <Arduino.h>
#include "motor_control.h"
#include "lidar_sensor.h"

unsigned long lastLidarRead = 0;

void setup()
{
  Serial.begin(9600);
  setupMotors();
  setupLidar();
  Serial.println("System ready");
}

void loop()
{
  // motor
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int pwmA, dirA, pwmB, dirB;
    if (sscanf(input.c_str(), "%d,%d,%d,%d", &pwmA, &dirA, &pwmB, &dirB) == 4)
    {
      setMotor(pwmA, dirA, pwmB, dirB);
    }
  }
  
  //lidar
  if (millis() - lastLidarRead > 100) {
    int distance = getDistance();
    Serial.print("L:");
    Serial.println(distance);
    lastLidarRead = millis();
  }
}