#ifndef DCMOTOR_HPP
#define DCMOTOR_HPP

#include <Arduino.h>

class DCMotor {
    private:
        int pwmPin; // PWM pin for motor speed control
        int dirAPin;
        int dirBPin;

    public:
        int encoderAPin;
        int encoderBPin;
        double encoderValue; // Encoder value

        DCMotor(int pwm, int dirA, int dirB, int encoderA, int encoderB);
        void setMotor(int analogSpeed);
        void setSpeed(int analogSpeed);
        void setDirection(int dir);
        void encoderSubroutineA();
        void encoderSubroutineB();
};

#endif // DCMOTOR_HPP