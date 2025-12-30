#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

class KalmanFilter {
private:
    float e_hat;      
    float e_dot_hat;  

    float P11, P12, P21, P22;

    float Q11; 
    float Q22; 
    float R;   
    float Kt;  

public:
    KalmanFilter(float q = 0.0, float r = 0.0, float kt = 0.0);

    float update(float rawError, float dt);

    void reset();

    void setParameters(float q, float r, float kt);
    
    float getEstimate() { return e_hat; }
};

#endif