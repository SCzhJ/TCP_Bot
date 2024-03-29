#include <Arduino.h>
#include <PID_v1.h>
#include "header.hpp"
#include "Mecanum_Kinematics.hpp"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "Wire.h"
// #include <MPU6050_light.h>

// MPU6050 mpu(Wire);

const float DeltaTime = 0.01; // in s, CANT USE MILLIS(), CONFLICT WITH PID lib
unsigned long last_time = 0; unsigned long curr_time = 0;
double dt = 0.04;

double Ceps = 0.0;  const float publish_dt = 0.1; float acc_t = 0;
double set_eps1 = EPRA*Ceps, set_eps2 = EPRB*Ceps, set_eps3 = EPRC*Ceps, set_eps4 = EPRD*Ceps;
float yaw_angle = 0;

PID motorPID1(&eps1_fb, &power1, &set_eps1, 0.05, 0.15, 0.001, DIRECT);
PID motorPID2(&eps2_fb, &power2, &set_eps2, 0.05, 0.14, 0.001, DIRECT);
PID motorPID3(&eps3_fb, &power3, &set_eps3, 0.05, 0.11, 0.001, DIRECT);
PID motorPID4(&eps4_fb, &power4, &set_eps4, 0.05, 0.140, 0.001, DIRECT);

MecanumDrive mecanumDrive(0.041, 0.105, 0.08);

// ROS elements
ros::NodeHandle nh;

std_msgs::Float32 distR; ros::Publisher distRPub("distR", &distR);
std_msgs::Float32 distL; ros::Publisher distLPub("distL", &distL);

std_msgs::Float32 x; std_msgs::Float32 y; std_msgs::Float32 theta;
ros::Publisher xPub("x", &x); ros::Publisher yPub("y", &y); ros::Publisher thetaPub("theta", &theta);

geometry_msgs::Twist prev_vel;
void cmd_vel_cb(const geometry_msgs::Twist& vel_msg);
ros::Subscriber<geometry_msgs::Twist> SubVel("cmd_vel", &cmd_vel_cb);

// Functions
void set_powers(double dt); void publish_topics(); 
void updateOdometry(double dt);
void measure_distance_right(); void measure_distance_left();

/************************************************/
/*                  Setups                      */
/************************************************/
void setup() {
    nh.initNode(); 
    nh.advertise(xPub); nh.advertise(yPub); nh.advertise(thetaPub);
    nh.advertise(distRPub); nh.advertise(distLPub);
    nh.subscribe(SubVel);

    attach_interrupts();
    motorPID1.SetMode(AUTOMATIC); motorPID2.SetMode(AUTOMATIC); motorPID3.SetMode(AUTOMATIC); motorPID4.SetMode(AUTOMATIC); 
    motorPID1.SetOutputLimits(-255, 255); motorPID2.SetOutputLimits(-255, 255); motorPID3.SetOutputLimits(-255, 255); motorPID4.SetOutputLimits(-255, 255);

    pinMode(echoPinR, INPUT); pinMode(trigPinR, OUTPUT);
    pinMode(echoPinL, INPUT); pinMode(trigPinL, OUTPUT);
    last_time = micros();
    curr_time = micros();
}

/************************************************/
/*               Control Loop                   */
/************************************************/
void loop() {
    curr_time = micros(); 
    dt = (curr_time - last_time)/1000000.0;
    last_time = curr_time;
    // Motor control
    set_powers(dt); 
    //updates
    updateOdometry(dt); 
    // ROS control + interval measure dist
    publish_topics();
    measure_distance_left();
    for (int i=0; i<4; i++){ nh.spinOnce(); delay(80*DeltaTime);}
    measure_distance_right();
    for (int i=0; i<4; i++){ nh.spinOnce(); delay(80*DeltaTime);}
}

/************************************************/
/*             Defined Functions                */
/************************************************/
void set_powers(double dt){
    eps1 = motor1.encoderValue / dt;
    eps2 = motor2.encoderValue / dt;
    eps3 = motor3.encoderValue / dt;
    eps4 = motor4.encoderValue / dt;

    eps1_fb = filter1.addValue(-eps1); eps2_fb = filter2.addValue(eps2); eps3_fb = filter3.addValue(-eps3); eps4_fb = filter4.addValue(eps4);
    motorPID1.Compute();motorPID2.Compute();motorPID3.Compute();motorPID4.Compute();
    motor1.setMotor(-power1);motor2.setMotor(power2);motor3.setMotor(-power3);motor4.setMotor(power4);
    motor1.encoderValue = 0;motor2.encoderValue = 0; motor3.encoderValue = 0;motor4.encoderValue = 0;
}

void cmd_vel_cb(const geometry_msgs::Twist& vel_msg){
    if (!(vel_msg.linear.x == prev_vel.linear.x && 
    vel_msg.linear.y == prev_vel.linear.y && 
    vel_msg.angular.z == prev_vel.angular.z) &&
    vel_msg.angular.x == 0){
        // if (power1>0){motor1.setMotor(-3);}
        // else if (power1<0){motor1.setMotor(3);}
        // if (power2>0){motor2.setMotor(-3);}
        // else if (power2<0){motor2.setMotor(3);}
        // if (power3>0){motor3.setMotor(-3);}
        // else if (power3<0){motor3.setMotor(3);}
        // if (power4>0){motor4.setMotor(-3);}
        // else if (power4<0){motor4.setMotor(3);}
        delay(5);
        motorPID1.outputSum = 0;
        motorPID2.outputSum = 0;
        motorPID3.outputSum = 0;
        motorPID4.outputSum = 0;
        motor1.setDirection(0);motor2.setDirection(0);motor3.setDirection(0);motor4.setDirection(0);
    }
    mecanumDrive.calculateWheelSpeeds(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
    set_eps1 = EPRA * mecanumDrive.wheelSpeeds[0] / 2 / pi; 
    set_eps2 = EPRB * mecanumDrive.wheelSpeeds[1] / 2 / pi; 
    set_eps3 = EPRC * mecanumDrive.wheelSpeeds[2] / 2 / pi; 
    set_eps4 = EPRD * mecanumDrive.wheelSpeeds[3] / 2 / pi;
    prev_vel = vel_msg;
}
void publish_topics(){
    xPub.publish(&x); yPub.publish(&y); thetaPub.publish(&theta);
    distRPub.publish(&distR); distLPub.publish(&distL);
    // LDRRPub.publish(&LDRR); LDRLPub.publish(&LDRL);
}
void updateOdometry(double dt){
    mecanumDrive.updateOdom(-eps1/EPRA * 2 * pi, eps2/EPRB * 2 * pi, -eps3/EPRC * 2 * pi, eps4/EPRD * 2 * pi, dt);
    x.data = mecanumDrive.odom[0];
    y.data = mecanumDrive.odom[1];
    theta.data = mecanumDrive.odom[2];
}
void measure_distance_right(){
    digitalWrite(trigPinR, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinR, LOW);
    durationR = pulseIn(echoPinR, HIGH);
    distR.data = (durationR / 2.0) / 29.1; // dist in cm

}
void measure_distance_left(){
    digitalWrite(trigPinL, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinL, LOW);
    durationL = pulseIn(echoPinL, HIGH);
    distL.data = (durationL / 2.0) / 29.1; // dist in cm
}