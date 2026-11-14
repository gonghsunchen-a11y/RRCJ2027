#include <Arduino.h>
#include <core_act.h>

void setup(){
    core_act_init();
    Serial3.setTimeout(200); // Allow camera image processing before its reply.
}

void loop(){
    //MotorStop(); // Stop while waiting for a fresh camera reply.
    readTopMaix_polar();
    update_gyro_sensor();

    if (!topmaixData.valid) {
        Serial.println("Camera packet missing or invalid");
    } else if (!topmaixData.goal_found) {
        Serial.println("No goal detected");
    } 
    else {
        Serial.printf("Angle: %u deg, distance: %u\n",
                      topmaixData.goal_angle,
                      topmaixData.goal_dist);
        float vx = 0.0f;
        float vy = 0.0f;
        int angleError = (int)topmaixData.goal_angle - 275;
        if (angleError > 180) angleError -= 360;
        if (angleError < -180) angleError += 360;

        // 235: robot is right of goal; 310: robot is left of goal.
        if (angleError < -5) {
            vx = -20.0f;
        } else if (angleError > 5) {
            vx = 20.0f;
        }
        if (topmaixData.goal_dist < 25) {
            vy = 20.0f;
        } else if (topmaixData.goal_dist > 45) {
            vy = -20.0f;
        }
        Vector_Motion(vx, vy, 0);
    }
}
