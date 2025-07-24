#include <Arduino.h>
#include <LIDARLite.h>
#include "lidar_sensor.h"

LIDARLite lidarLite;

void setupLidar() {
    lidarLite.begin(0, true);
}

int getDistance() {
    return lidarLite.distance();
}