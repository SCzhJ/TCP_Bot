#include <Arduino.h>
#include "DC_Motor.hpp"

DCMotor::DCMotor(int pwm, int dirA, int dirB, int encoderA, int encoderB) : 
pwmPin(pwm), dirAPin(dirA), dirBPin(dirB), encoderAPin(encoderA),
encoderBPin(encoderB), encoderValue(0) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirAPin, OUTPUT);
    pinMode(dirBPin, OUTPUT);
    pinMode(encoderAPin, INPUT);
    pinMode(encoderBPin, INPUT);
}
void DCMotor::setMotor(int analogSpeed){
    if (analogSpeed > 0){
        setDirection(1);
        setSpeed(analogSpeed);
    }
    else if (analogSpeed <= 0){
        setDirection(-1);
        setSpeed(analogSpeed);
    }
    else{
        setDirection(0);
    }
    
}
void DCMotor::setSpeed(int analogSpeed) {
    if (analogSpeed < 0) {
        analogSpeed = 0;
    } 
    else if (analogSpeed > 225) {
        analogSpeed = 225;
    }
    analogWrite(pwmPin, analogSpeed);
}
void DCMotor::setDirection(int dir) {
    // 1 forward, 0 stop, -1 backward
    if (dir == 1) {
        digitalWrite(dirAPin, HIGH);
        digitalWrite(dirBPin, LOW);
    } 
    else if(dir == 0) {
        digitalWrite(dirAPin, LOW);
        digitalWrite(dirBPin, LOW);
    }
    else if(dir == -1){
        digitalWrite(dirAPin, LOW);
        digitalWrite(dirBPin, HIGH);
    }
    else {
        Serial.println("Invalid direction");
        digitalWrite(dirAPin, LOW);
        digitalWrite(dirBPin, LOW);
    }
}
void DCMotor::encoderSubroutine() {
    // This function will be called in the interrupt service routine
    // to update the encoder value
    if (digitalRead(encoderBPin)){
        encoderValue ++;
    }
    else{ 
        encoderValue --;
    }
}