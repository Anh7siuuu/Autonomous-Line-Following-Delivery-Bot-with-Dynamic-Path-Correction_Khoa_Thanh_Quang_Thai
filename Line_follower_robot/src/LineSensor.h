#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "SharedData.h"

class LineSensor {
private:
    int pinL2, pinL1, pinR1, pinR2;
    int lastValidSum = 0; 

public:
    LineSensor(int l2, int l1, int r1, int r2);
    void begin();
    float computeError(); 
    float computeErrorWithKalman();
};

#endif