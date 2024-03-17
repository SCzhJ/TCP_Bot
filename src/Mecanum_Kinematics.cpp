#include <Arduino.h>
#include "Mecanum_Kinematics.hpp"


MecanumDrive::MecanumDrive(double wheelRadius, double robotLength, double robotWidth)
: wheelRadius(wheelRadius), robotLength(robotLength), robotWidth(robotWidth){
    lpw = robotLength + robotWidth;
    odom[0] = 0; odom[1] = 0; odom[2] = 0;
}

void MecanumDrive::calculateWheelSpeeds(double vx, double vy, double w) {
    wheelSpeeds[0] = (1 / wheelRadius) * (vx - vy - lpw * w); // Front left wheel speed
    wheelSpeeds[1] = (1 / wheelRadius) * (vx + vy + lpw * w); // Front right wheel speed
    wheelSpeeds[2] = (1 / wheelRadius) * (vx + vy - lpw * w); // Rear left wheel speed
    wheelSpeeds[3] = (1 / wheelRadius) * (vx - vy + lpw * w); // Rear right wheel speed
}

void MecanumDrive::updateOdom(float wfl, float wfr, float wrl, float wrr, float dt) {
    float delta_x = (wheelRadius / 4) * (wfl + wfr + wrl + wrr);
    float delta_y = (wheelRadius / 4) * (-wfl + wfr + wrl - wrr);
    float delta_theta = (wheelRadius / (4 * lpw)) * (-wfl + wfr - wrl + wrr);
    odom[0] += delta_x * dt * cos(odom[2]) - delta_y * dt * sin(odom[2]);
    odom[1] += delta_x * dt*  sin(odom[2]) + delta_y * dt * cos(odom[2]);
    odom[2] += delta_theta * dt;
    while (odom[2] >= 2*3.14159265358979323846){
        odom[2] -= 2*3.14159265358979323846;
    }
    while (odom[2] < 0){
        odom[2] += 2*3.14159265358979323846;
    }

}