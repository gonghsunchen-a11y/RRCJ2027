#include <Arduino.h>
#include <core_act.h>
#include <math.h>

// 0：正常，1：前進到 320°，2：左右移到 270°
int returnStage = 0;

void setup(){
    core_act_init();
    Serial3.setTimeout(200);
}

void loop(){
    //MotorStop();
    readTopMaix_polar();
    update_gyro_sensor();

    if (!topmaixData.valid) {
        Serial.println("Camera packet missing or invalid");
    } else if (!topmaixData.goal_found) {
        Serial.println("No goal detected");
    } else {
        Serial.printf("Angle: %u deg, distance: %u\n",
                      topmaixData.goal_angle,
                      topmaixData.goal_dist);

        int angle = topmaixData.goal_angle;
        int distance = topmaixData.goal_dist;

        if (returnStage == 0 && angle > 325 && distance > 70) {
            returnStage = 1;
        }

        if (returnStage == 1) {
            if (angle > 320) {
                Vector_Motion(0, 20, 0);
                return;
            }
            returnStage = 2;
        }

        int angleError = angle - 270;
        if (angleError > 180) angleError -= 360;
        if (angleError < -180) angleError += 360;

        float speed;

        if (abs(angleError) > 12) {
            // 兩側：越接近 270° 越慢
            speed = constrain(
                12.0f + (abs(angleError) - 12) * 0.1f,
                12.0f, 20.0f
            );
        } else {
            // 中間：越接近 distance 28 越慢
            speed = constrain(
                12.0f + abs(distance - 28) * 0.2f,
                12.0f, 20.0f
            );
        }

        if (returnStage == 2) {
            if (abs(angleError) > 5) {
                // 此階段只左右移動，依角度減速
                float sideSpeed = constrain(
                    12.0f + (abs(angleError) - 5) * 0.1f,
                    12.0f, 20.0f
                );

                Vector_Motion(
                    angleError > 0 ? sideSpeed : -sideSpeed,
                    0, 0
                );
                return;
            }
            returnStage = 0;
        }

        float vx = 0.0f;
        float vy = 0.0f;

        if (angleError < -5) {
            vx = -speed;
        } else if (angleError > 5) {
            vx = speed;
        }

        if (distance < 25) {
            vy = speed;
        } else if (distance > 45) {
            vy = -speed;
        }

        // 斜向移動時，合成速度維持 speed
        if (vx != 0 && vy != 0) {
            vx *= 0.7071f;
            vy *= 0.7071f;
        }

        Vector_Motion(vx, vy, 0);
    }
}