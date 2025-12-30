#include "MotorPID.h"

MotorPID::MotorPID(int ena, int in1, int in2, int enb, int in3, int in4) {
    pinENA = ena; 
    pinIN1 = in1; 
    pinIN2 = in2;
    pinENB = enb; 
    pinIN3 = in3; 
    pinIN4 = in4;
}

void MotorPID::begin() {
    pinMode(pinENA, OUTPUT); 
    pinMode(pinIN1, OUTPUT); 
    pinMode(pinIN2, OUTPUT);
    pinMode(pinENB, OUTPUT); 
    pinMode(pinIN3, OUTPUT); 
    pinMode(pinIN4, OUTPUT);
}

void MotorPID::stop() {
    driveHardware(0, 0);
    lastError = 0; 
    integral = 0;
    shared.pwmLeft = 0; 
    shared.pwmRight = 0;
}

void MotorPID::computeAndDrive(float error, float dt) {
    integral += error;
    integral = constrain(integral, -100, 100);

    float derivative = (error - lastError) / dt;
    float output = (shared.Kp * error) + (shared.Ki * integral) + (shared.Kd * derivative);
    
    lastError = error;
    shared.pidOutput = output;

    int speedLeft = shared.baseSpeed + output;
    int speedRight = shared.baseSpeed - output;

    speedLeft = constrain(speedLeft, 0, shared.maxSpeed);
    speedRight = constrain(speedRight, 0, shared.maxSpeed);

    shared.pwmLeft = speedLeft;
    shared.pwmRight = speedRight;
    driveHardware(speedLeft, speedRight);
}

void MotorPID::driveHardware(int left, int right) {
    digitalWrite(pinIN1, HIGH); 
    digitalWrite(pinIN2, LOW);
    digitalWrite(pinIN3, HIGH); 
    digitalWrite(pinIN4, LOW);
    analogWrite(pinENA, left);
    analogWrite(pinENB, right);
}