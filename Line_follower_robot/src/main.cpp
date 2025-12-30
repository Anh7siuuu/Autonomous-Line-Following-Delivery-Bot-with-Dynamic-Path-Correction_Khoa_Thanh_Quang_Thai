#include <Arduino.h>
#include "SharedData.h"
#include "LineSensor.h"
#include "MotorPID.h"
#include "KalmanFilter.h"
#include "WebDashboard.h"

SharedData shared; 
KalmanFilter kalman(0, 0, 0);

LineSensor sensors(17, 35, 16, 34); 
MotorPID motors(25, 32, 33, 26, 14, 27); 
WebDashboard dashboard("XE_DO_LINE", "12345678");
TaskHandle_t TaskPIDHandle;

    void TaskPID(void * pvParameters) {
        static uint32_t lastMicros = micros();
        sensors.begin();
        motors.begin();
        for(;;) {
            uint32_t now = micros();
            float dt = (now - lastMicros) * 1e-6;
            lastMicros = now;
            if (shared.isRunning) {
                float error = sensors.computeErrorWithKalman();
                shared.currentError = error; 
                kalman.setParameters(shared.Q, shared.R, shared.Kt);
                float newError = kalman.update(error, dt);
                motors.computeAndDrive(newError, dt);
                
            } else {
                motors.stop();
                kalman.reset();
            }
            float dt_temp = dt * 1e6;
            shared.loopTime = (int)dt_temp;
            vTaskDelay(1 / portTICK_PERIOD_MS); 
        }
    }

void setup() {
    Serial.begin(115200);
    dashboard.begin();
    xTaskCreatePinnedToCore(
        TaskPID,        
        "PID_Task",     
        4096,           
        NULL,           
        1,              
        &TaskPIDHandle, 
        1               
    );
}

void loop() {
    dashboard.cleanup(); 
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 100) { 
        dashboard.notifyClients();
        lastTime = millis();
    }
}
