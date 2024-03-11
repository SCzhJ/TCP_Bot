#include <Arduino.h>
#include <PID_v1.h>
#include "DC_Motor.hpp"
#include "constants.hpp"

DCMotor motor1(pwm1, dir1A, dir1B, encoder1A, encoder1B);
DCMotor motor2(pwm2, dir2A, dir2B, encoder2A, encoder2B);
DCMotor motor3(pwm3, dir3A, dir3B, encoder3A, encoder3B);
DCMotor motor4(pwm4, dir4A, dir4B, encoder4A, encoder4B);
void encoder_subroutine_1();void encoder_subroutine_2();void encoder_subroutine_3();void encoder_subroutine_4();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(encoder1A), encoder_subroutine_1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2A), encoder_subroutine_2, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder3A), encoder_subroutine_3, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder4A), encoder_subroutine_4, RISING);
}

void loop() {
    if (Serial.available() > 0) {
      // Read the incoming byte
      String incomingMessage = Serial.readStringUntil('\n');
      // Echo the message back to the Serial Monitor
      Serial.print("I received: ");
      Serial.println(incomingMessage);
      while(1){
          // put your main code here, to run repeatedly:
          // motor1.setDirection(1);
          // motor1.setSpeed(200);
          // motor2.setDirection(1);
          // motor2.setSpeed(200);
          // motor3.setDirection(1);
          // motor3.setSpeed(200);
          // motor4.setDirection(1);
          // motor4.setSpeed(200);
          delay(100);
          // Serial.print("motor1 encoder value:");
          // Serial.println(motor1.encoderValue);
          Serial.print("motor2 encoder value:");
          Serial.println(motor2.encoderValue);
          // Serial.print("motor3 encoder value:");
          // Serial.println(motor3.encoderValue);
          // Serial.print("motor4 encoder value:");
          // Serial.println(motor4.encoderValue);
      }
    }
    delay(500);

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