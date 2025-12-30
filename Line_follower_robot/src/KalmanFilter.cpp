#include "KalmanFilter.h"

// Constructor
KalmanFilter::KalmanFilter(float q, float r, float kt) {
    Q11 = q;
    Q22 = 0.1; 
    R = r;
    Kt = kt;
    reset(); 
}

void KalmanFilter::reset() {
    e_hat = 0;
    e_dot_hat = 0;
    P11 = 1; 
    P12 = 0;
    P21 = 0; 
    P22 = 1;
}

void KalmanFilter::setParameters(float q, float r, float kt) {
    Q11 = q;
    R = r;
    Kt = kt;
}

float KalmanFilter::update(float z, float dt) {
    float e_pred     = e_hat + dt * e_dot_hat;
    float e_dot_pred = e_dot_hat; 

    float P11_pred = P11 + dt*(P21 + P12) + dt*dt*P22 + Q11;
    float P12_pred = P12 + dt*P22;
    float P21_pred = P21 + dt*P22;
    float P22_pred = P22 + Q22;

    float S = P11_pred + R;
    float K1 = P11_pred / S; 
    float K2 = P21_pred / S; 

    float y = z - e_pred;

    e_hat     = e_pred     + K1 * y;
    e_dot_hat = e_dot_pred + K2 * y;

    P11 = (1 - K1) * P11_pred;
    P12 = (1 - K1) * P12_pred;
    P21 = -K2 * P11_pred + P21_pred;
    P22 = -K2 * P12_pred + P22_pred;

    return e_hat + (Kt * e_dot_hat);
}