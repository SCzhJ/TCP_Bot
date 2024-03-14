#pragma once
#include <Arduino.h>
#include "util.hpp"
#include "constants.hpp"

const double EPRA = 325;//转速比：1：(320-330)
const double EPRB = 330;//转速比：1：(325-330)
const double EPRC = 332.5;//转速比：1：(330-335)
const double EPRD = 332.5;//转速比：1：(335-330)

const int pwm1 = 12; const int dir1A = 34; const int dir1B = 35; const int encoder1A = 18; const int encoder1B = 31;
const int pwm2 = 8; const int dir2A = 37; const int dir2B = 36; const int encoder2A = 19; const int encoder2B = 38;
const int pwm3 = 6; const int dir3A = 43; const int dir3B = 42; const int encoder3A = 3; const int encoder3B = 49;
const int pwm4 = 5; const int dir4A = A4; const int dir4B = A5; const int encoder4A = 2; const int encoder4B = A1;

const double pi = 3.14159265358979323846;

double eps1 = 0, eps2 = 0, eps3 = 0, eps4 = 0;// encoder count per second
double eps1_fb = 0, eps2_fb = 0, eps3_fb = 0, eps4_fb = 0;// eps filtered feedback
double power1 = 10, power2 = 10, power3 = 10, power4 = 10;

MovingAverage filter1;MovingAverage filter2;MovingAverage filter3;MovingAverage filter4;

DCMotor motor1(pwm1, dir1A, dir1B, encoder1A, encoder1B);DCMotor motor2(pwm2, dir2A, dir2B, encoder2A, encoder2B);
DCMotor motor3(pwm3, dir3A, dir3B, encoder3A, encoder3B);DCMotor motor4(pwm4, dir4A, dir4B, encoder4A, encoder4B);

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