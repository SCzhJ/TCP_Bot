#include <Arduino.h>
#include <PID_v1.h>
#include "DC_Motor.hpp"
#include "constants.hpp"
#include "util.hpp"
#include "variables.hpp"
#include <ros.h>
#include <std_msgs/Float32.h>


double Ceps = 0; const float DeltaTime = 0.04; // in s
double set_eps1 = EPRA*Ceps, set_eps2 = EPRB*Ceps, set_eps3 = EPRC*Ceps, set_eps4 = EPRD*Ceps;
double prev_set1 = 0, prev_set2 = 0, prev_set3 = 0, prev_set4 = 0;

PID motorPID1(&eps1_fb, &power1, &set_eps1, 0.05, 0.15, 0.001, DIRECT);
PID motorPID2(&eps2_fb, &power2, &set_eps2, 0.05, 0.14, 0.001, DIRECT);
PID motorPID3(&eps3_fb, &power3, &set_eps3, 0.05, 0.11, 0.001, DIRECT);
PID motorPID4(&eps4_fb, &power4, &set_eps4, 0.05, 0.140, 0.001, DIRECT);

// ROS elements
ros::NodeHandle nh;
std_msgs::Float32 rd_w1; std_msgs::Float32 rd_w2; std_msgs::Float32 rd_w3; std_msgs::Float32 rd_w4;
ros::Publisher motor1_reading("rd_w1", &rd_w1);ros::Publisher motor2_reading("rd_w2", &rd_w2);ros::Publisher motor3_reading("rd_w3", &rd_w3);ros::Publisher motor4_reading("rd_w4", &rd_w4);
void w1_cb(const std_msgs::Float32& w1_msg){
    set_eps1 = EPRA * w1_msg.data/(2*pi); 
    if (prev_set1 != set_eps1){
        motor1.setDirection(0);
        motor2.setSpeed(0);
        motorPID1.SetMode(MANUAL);
        motorPID1.SetMode(AUTOMATIC);
        delay(1000*DeltaTime);
    }
    prev_set1 = set_eps1;
}
void w2_cb(const std_msgs::Float32& w2_msg){
    set_eps2 = EPRB * w2_msg.data/(2*pi); 
    if (prev_set2 != set_eps2){
        motor2.setDirection(0);
        motor2.setSpeed(0);
        motorPID2.SetMode(MANUAL);
        motorPID2.SetMode(AUTOMATIC);
        delay(1000*DeltaTime);
    }
    prev_set2 = set_eps2;
}
void w3_cb(const std_msgs::Float32& w3_msg){
    set_eps3 = EPRC * w3_msg.data/(2*pi); 
    if (prev_set3 != set_eps3){
        motor3.setDirection(0);
        motor3.setSpeed(0);
        motorPID3.SetMode(MANUAL);
        motorPID3.SetMode(AUTOMATIC);
        delay(1000*DeltaTime);
    }
    prev_set3 = set_eps3;
}
void w4_cb(const std_msgs::Float32& w4_msg){
    set_eps4 = EPRD * w4_msg.data/(2*pi); 
    if (prev_set4 != set_eps4){
        motor4.setDirection(0);
        motor4.setSpeed(0);
        motorPID4.SetMode(MANUAL);
        motorPID4.SetMode(AUTOMATIC);
        delay(1000*DeltaTime);
    }
    prev_set4 = set_eps4;
} 
ros::Subscriber<std_msgs::Float32> Sub_AngVel1("w1", &w1_cb);
ros::Subscriber<std_msgs::Float32> Sub_AngVel2("w2", &w2_cb);
ros::Subscriber<std_msgs::Float32> Sub_AngVel3("w3", &w3_cb);
ros::Subscriber<std_msgs::Float32> Sub_AngVel4("w4", &w4_cb);


void display_eps();
void set_powers();

void setup() {
    // put your setup code here, to run once:
    nh.initNode();
    Serial.begin(115200);
    nh.advertise(motor1_reading); nh.advertise(motor2_reading); nh.advertise(motor3_reading); nh.advertise(motor4_reading);
    nh.subscribe(Sub_AngVel1); nh.subscribe(Sub_AngVel2); nh.subscribe(Sub_AngVel3); nh.subscribe(Sub_AngVel4);
    attachInterrupt(digitalPinToInterrupt(encoder1A), encoder_subroutine_1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2A), encoder_subroutine_2, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder3A), encoder_subroutine_3, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder4A), encoder_subroutine_4, RISING);
    motorPID1.SetMode(AUTOMATIC); motorPID2.SetMode(AUTOMATIC); motorPID3.SetMode(AUTOMATIC); motorPID4.SetMode(AUTOMATIC); 
    motorPID1.SetOutputLimits(-255, 255); motorPID2.SetOutputLimits(-255, 255); motorPID3.SetOutputLimits(-255, 255); motorPID4.SetOutputLimits(-255, 255);
}

void loop() {
    set_powers();

    rd_w1.data = -eps1/EPRA * 2 * pi; rd_w2.data = eps2/EPRB * 2 * pi; rd_w3.data = -eps3/EPRC * 2 * pi; rd_w4.data = eps4/EPRD * 2 * pi;
    // display_eps();

    motor1_reading.publish(&rd_w1); motor2_reading.publish(&rd_w2); motor3_reading.publish(&rd_w3); motor4_reading.publish(&rd_w4);
    nh.spinOnce();
    delay(DeltaTime*1000);
}

void set_powers(){
    eps1 = motor1.encoderValue / DeltaTime;
    eps1_fb = filter1.addValue(-eps1);
    motorPID1.Compute();
    motor1.setMotor(-power1);

    eps2 = motor2.encoderValue / DeltaTime;
    eps2_fb = filter2.addValue(eps2);
    motorPID2.Compute();
    motor2.setMotor(power2);

    eps3 = motor3.encoderValue / DeltaTime;
    eps3_fb = filter3.addValue(-eps3);
    motorPID3.Compute();
    motor3.setMotor(-power3);

    eps4 = motor4.encoderValue / DeltaTime;
    eps4_fb = filter4.addValue(eps4);
    motorPID4.Compute();
    motor4.setMotor(power4);

    motor1.encoderValue = 0;
    motor2.encoderValue = 0; 
    motor3.encoderValue = 0;
    motor4.encoderValue = 0;
}

void display_eps(){
    Serial.print(-eps1);
    Serial.print(" ");
    Serial.print(eps2);
    Serial.print(" ");
    Serial.print(-eps3);
    Serial.print(" ");
    Serial.print(eps4);
    Serial.println();
}