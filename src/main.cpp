#include <Arduino.h>
#include <PID_v1.h>
#include "header.hpp"
#include "Mecanum_Kinematics.hpp"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

double Ceps = 0.0; 
const float DeltaTime = 0.04; // in s
double dt; double prev_t; double curr_t;
double set_eps1 = EPRA*Ceps, set_eps2 = EPRB*Ceps, set_eps3 = EPRC*Ceps, set_eps4 = EPRD*Ceps;
geometry_msgs::Twist prev_vel;

PID motorPID1(&eps1_fb, &power1, &set_eps1, 0.05, 0.15, 0.001, DIRECT);
PID motorPID2(&eps2_fb, &power2, &set_eps2, 0.05, 0.14, 0.001, DIRECT);
PID motorPID3(&eps3_fb, &power3, &set_eps3, 0.05, 0.11, 0.001, DIRECT);
PID motorPID4(&eps4_fb, &power4, &set_eps4, 0.05, 0.140, 0.001, DIRECT);

MecanumDrive mecanumDrive(0.036, 0.105, 0.08);

// ROS elements
ros::NodeHandle nh;
std_msgs::Float32 rd_w1; std_msgs::Float32 rd_w2; std_msgs::Float32 rd_w3; std_msgs::Float32 rd_w4;
ros::Publisher motor1_reading("rd_w1", &rd_w1);ros::Publisher motor2_reading("rd_w2", &rd_w2);ros::Publisher motor3_reading("rd_w3", &rd_w3);ros::Publisher motor4_reading("rd_w4", &rd_w4);

void cmd_vel_cb(const geometry_msgs::Twist& vel_msg);
ros::Subscriber<geometry_msgs::Twist> SubVel("cmd_vel", &cmd_vel_cb);

void display_eps();
void set_powers();

void setup() {
    // put your setup code here, to run once:
    nh.initNode();
    Serial.begin(115200);
    nh.advertise(motor1_reading); nh.advertise(motor2_reading); nh.advertise(motor3_reading); nh.advertise(motor4_reading);
    nh.subscribe(SubVel);
    attachInterrupt(digitalPinToInterrupt(encoder1A), encoder_subroutine_1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2A), encoder_subroutine_2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder3A), encoder_subroutine_3A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder4A), encoder_subroutine_4A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1B), encoder_subroutine_1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2B), encoder_subroutine_2B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder3B), encoder_subroutine_3B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder4B), encoder_subroutine_4B, CHANGE);
    motorPID1.SetMode(AUTOMATIC); motorPID2.SetMode(AUTOMATIC); motorPID3.SetMode(AUTOMATIC); motorPID4.SetMode(AUTOMATIC); 
    motorPID1.SetOutputLimits(-255, 255); motorPID2.SetOutputLimits(-255, 255); motorPID3.SetOutputLimits(-255, 255); motorPID4.SetOutputLimits(-255, 255);
    prev_t = millis();
    curr_t = millis();
}

void loop() {
    set_powers();
    // display_eps();

    rd_w1.data = -eps1/EPRA * 2 * pi; rd_w2.data = eps2/EPRB * 2 * pi; rd_w3.data = -eps3/EPRC * 2 * pi; rd_w4.data = eps4/EPRD * 2 * pi;
    motor1_reading.publish(&rd_w1); motor2_reading.publish(&rd_w2); motor3_reading.publish(&rd_w3); motor4_reading.publish(&rd_w4);
    nh.spinOnce();
    delay(DeltaTime*1000);
}

void set_powers(){
    eps1 = motor1.encoderValue / DeltaTime;
    eps2 = motor2.encoderValue / DeltaTime;
    eps3 = motor3.encoderValue / DeltaTime;
    eps4 = motor4.encoderValue / DeltaTime;

    eps1_fb = filter1.addValue(-eps1); eps2_fb = filter2.addValue(eps2); eps3_fb = filter3.addValue(-eps3); eps4_fb = filter4.addValue(eps4);
    motorPID1.Compute();motorPID2.Compute();motorPID3.Compute();motorPID4.Compute();
    motor1.setMotor(-power1);motor2.setMotor(power2);motor3.setMotor(-power3);motor4.setMotor(power4);
    motor1.encoderValue = 0;motor2.encoderValue = 0; motor3.encoderValue = 0;motor4.encoderValue = 0;
}

void display_eps(){
    Serial.print(eps1_fb);
    Serial.print(" ");
    Serial.print(eps2_fb);
    Serial.print(" ");
    Serial.print(eps3_fb);
    Serial.print(" ");
    Serial.println(eps4_fb);
}
void cmd_vel_cb(const geometry_msgs::Twist& vel_msg){
    if (!(vel_msg.linear.x == prev_vel.linear.x && 
    vel_msg.linear.y == prev_vel.linear.y && 
    vel_msg.angular.z == prev_vel.angular.z)){
        if (power1>0){motor1.setMotor(-10);}
        else if (power1<0){motor1.setMotor(10);}
        if (power2>0){motor2.setMotor(-10);}
        else if (power2<0){motor2.setMotor(10);}
        if (power3>0){motor3.setMotor(-10);}
        else if (power3<0){motor3.setMotor(10);}
        if (power4>0){motor4.setMotor(-10);}
        else if (power4<0){motor4.setMotor(10);}
        motorPID1.outputSum = 0;
        motorPID2.outputSum = 0;
        motorPID3.outputSum = 0;
        motorPID4.outputSum = 0;
        delay(30);
        motor1.setDirection(0);motor2.setDirection(0);motor3.setDirection(0);motor4.setDirection(0);
    }
    mecanumDrive.calculateWheelSpeeds(vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
    set_eps1 = EPRA * mecanumDrive.wheelSpeeds[0] / 2 / pi; 
    set_eps2 = EPRB * mecanumDrive.wheelSpeeds[1] / 2 / pi; 
    set_eps3 = EPRC * mecanumDrive.wheelSpeeds[2] / 2 / pi; 
    set_eps4 = EPRD * mecanumDrive.wheelSpeeds[3] / 2 / pi;
    prev_vel = vel_msg;
}