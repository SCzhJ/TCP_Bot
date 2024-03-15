#ifndef MECANUM_KINEMATICS_HPP
#define MECANUM_KINEMATICS_HPP

#include <Arduino.h>

class MecanumDrive {
private:
    double wheelRadius; // Radius of the wheels
    double robotLength; // Distance between front and back wheels
    double robotWidth;  // Distance between left and right wheels
    double lpw;

public:
    MecanumDrive(double wheelRadius, double robotLength, double robotWidth);
    void calculateWheelSpeeds(double vx, double vy, double w);
    void updateOdom(float wfl, float wfr, float wrl, float wrr, float dt);
    double wheelSpeeds[4];
    float odom[3];
};

#endif