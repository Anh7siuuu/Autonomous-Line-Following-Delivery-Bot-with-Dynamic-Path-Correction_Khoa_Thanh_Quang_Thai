#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>
#include "SharedData.h"

class MotorPID {
private:
    int pinENA, pinIN1, pinIN2;
    int pinENB, pinIN3, pinIN4;
    float lastError = 0;
    float integral = 0;

    void driveHardware(int left, int right);

public:
    MotorPID(int ena, int in1, int in2, int enb, int in3, int in4);
    void begin();
    void stop();
    void computeAndDrive(float error, float dt);
};

#endif