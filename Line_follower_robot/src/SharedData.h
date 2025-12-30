#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>

struct SharedData {
    volatile float Kp = 35.0;
    volatile float Ki = 0.0;
    volatile float Kd = 20.0;
    volatile int baseSpeed = 140;
    volatile int maxSpeed = 180;
    volatile bool isRunning = false;
    bool HaveLine = false;

    volatile int sensorRaw[4] = {0, 0, 0, 0}; 
    volatile float currentError = 0.0;
    volatile float pidOutput = 0.0;
    volatile int pwmLeft = 0;
    volatile int pwmRight = 0;

    volatile float Q = 0.0;  
    volatile float R = 0.0;  
    volatile float Kt = 0.0; 

    volatile unsigned long loopTime = 0;  
    
};

extern SharedData shared;

#endif