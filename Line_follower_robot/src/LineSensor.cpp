#include "LineSensor.h"

LineSensor::LineSensor(int l2, int l1, int r1, int r2) {
    pinL2 = l2; 
    pinL1 = l1; 
    pinR1 = r1; 
    pinR2 = r2;
}

void LineSensor::begin() {
    pinMode(pinL2, INPUT); 
    pinMode(pinL1, INPUT);
    pinMode(pinR1, INPUT); 
    pinMode(pinR2, INPUT);
}

float LineSensor::computeError() {
    int L2 = digitalRead(pinL2);
    int L1 = digitalRead(pinL1);
    int R1 = digitalRead(pinR1);
    int R2 = digitalRead(pinR2);

    shared.sensorRaw[0] = L2; 
    shared.sensorRaw[1] = L1;
    shared.sensorRaw[2] = R1; 
    shared.sensorRaw[3] = R2;

    int sum = 0;
    int cnt = 0;

    if (L2) { sum += -3; cnt++; }
    if (L1) { sum += -1; cnt++; }
    if (R1) { sum += +1; cnt++; }
    if (R2) { sum += +3; cnt++; }

    if (cnt > 0) {
        shared.HaveLine = true;
        lastValidSum = sum; 
        return (float)sum;  
    } 
    else {
        shared.HaveLine = false;
        return (float)lastValidSum; 
    }
}

float LineSensor::computeErrorWithKalman() {
    int L2 = digitalRead(pinL2);
    int L1 = digitalRead(pinL1);
    int R1 = digitalRead(pinR1);
    int R2 = digitalRead(pinR2);

    shared.sensorRaw[0] = L2; 
    shared.sensorRaw[1] = L1;
    shared.sensorRaw[2] = R1; 
    shared.sensorRaw[3] = R2;

    int sum = 0;
    int cnt = 0;

    if (L2) { sum += -3; cnt++; }
    if (L1) { sum += -1; cnt++; }
    if (R1) { sum += +1; cnt++; }
    if (R2) { sum += +3; cnt++; }
   
    if (cnt > 0) {
        shared.HaveLine = true;
        lastValidSum = sum; 
        return (float)sum;  
    } 
    else {
        shared.HaveLine = false;
        return (float)lastValidSum; 
    } 

}