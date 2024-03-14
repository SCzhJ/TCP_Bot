#include <Arduino.h>
#include "Mecanum_Kinematics.hpp"


MecanumDrive::MecanumDrive(double wheelRadius, double robotLength, double robotWidth)
: wheelRadius(wheelRadius), robotLength(robotLength), robotWidth(robotWidth){
    lpw = robotLength + robotWidth;
}

void MecanumDrive::calculateWheelSpeeds(double vx, double vy, double w) {
    wheelSpeeds[0] = (1 / wheelRadius) * (vx - vy - lpw * w); // Front left wheel speed
    wheelSpeeds[1] = (1 / wheelRadius) * (vx + vy + lpw * w); // Front right wheel speed
    wheelSpeeds[2] = (1 / wheelRadius) * (vx + vy - lpw * w); // Rear left wheel speed
    wheelSpeeds[3] = (1 / wheelRadius) * (vx - vy + lpw * w); // Rear right wheel speed
}