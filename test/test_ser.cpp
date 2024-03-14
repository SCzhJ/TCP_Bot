#include <Arduino.h>
#include <PID_v1.h>
#include "DC_Motor.hpp"
#include "constants.hpp"
#include "util.hpp"
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup() {
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
}

DCMotor motor1(pwm1, dir1A, dir1B, encoder1A, encoder1B);
DCMotor motor2(pwm2, dir2A, dir2B, encoder2A, encoder2B);
DCMotor motor3(pwm3, dir3A, dir3B, encoder3A, encoder3B);
DCMotor motor4(pwm4, dir4A, dir4B, encoder4A, encoder4B);
void encoder_subroutine_1();void encoder_subroutine_2();void encoder_subroutine_3();void encoder_subroutine_4();

double eps1 = 0, eps2 = 0, eps3 = 0, eps4 = 0;// encoder count per second
double eps1_fb = 0, eps2_fb = 0, eps3_fb = 0, eps4_fb = 0;// eps filtered feedback
double set_eps1 = 700, set_eps2 = 700, set_eps3 = 0, set_eps4 = 0;
double power1 = 10, power2 = 10, power3 = 10, power4 = 10;
const float DeltaTime = 0.02; // in s
MovingAverage filter1;
MovingAverage filter2;
MovingAverage filter3;
MovingAverage filter4;

PID motorPID1(&eps1_fb, &power1, &set_eps1, 0.05, 0.25, 0.0015, DIRECT);
PID motorPID2(&eps2_fb, &power2, &set_eps2, 0.05, 0.1, 0.001, DIRECT);
PID motorPID3(&eps3_fb, &power3, &set_eps3, 0.5, 0.1, 0, DIRECT);
PID motorPID4(&eps4_fb, &power4, &set_eps4, 0.5, 0.1, 0, DIRECT);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(encoder1A), encoder_subroutine_1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2A), encoder_subroutine_2, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder3A), encoder_subroutine_3, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder4A), encoder_subroutine_4, RISING);
    motorPID1.SetMode(AUTOMATIC); // Turn on the PID controller
    motorPID2.SetMode(AUTOMATIC); // Turn on the PID controller
    motorPID3.SetMode(AUTOMATIC); // Turn on the PID controller
    motorPID4.SetMode(AUTOMATIC); // Turn on the PID controller
    motorPID1.SetOutputLimits(0, 255);
    motorPID2.SetOutputLimits(0, 255);
    motorPID3.SetOutputLimits(0, 255);
    motorPID4.SetOutputLimits(0, 255);

}

void loop() {

    eps1 = motor1.encoderValue / DeltaTime;
    eps1_fb = filter1.addValue(eps1);

    motorPID1.Compute();
    motor1.setMotor(power1);



    motor1.encoderValue = 0;


    eps2 = motor2.encoderValue / DeltaTime;
    eps2_fb = filter2.addValue(eps2);

    motorPID2.Compute();
    motor2.setMotor(power2);
    Serial.print(eps1);
    Serial.print(",");
    Serial.println(eps2);

    motor2.encoderValue = 0;
    delay(DeltaTime * 1000);

    //Serial.println(motor4.encoderValue);

}

void encoder_subroutine_1(){
  motor1.encoderSubroutine();
}
void encoder_subroutine_2(){
  motor2.encoderSubroutine();
}
void encoder_subroutine_3(){
  motor3.encoderSubroutine();
}
void encoder_subroutine_4(){
  motor4.encoderSubroutine();
}