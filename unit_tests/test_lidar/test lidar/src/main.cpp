#include <Arduino.h>
#include <LIDARLite.h>

LIDARLite lidarLite;

void setup() {
    Serial.begin(115200);
    lidarLite.begin(0, true);
    Serial.println("LIDAR initialized");
}

void loop() {
    int distance = lidarLite.distance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);
}