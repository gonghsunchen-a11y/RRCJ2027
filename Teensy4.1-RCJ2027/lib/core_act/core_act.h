#ifndef CORE_ACT_H
#define CORE_ACT_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Motor Pins (index = SetMotorSpeed port) ---
//               port:  1   2   3   4
#define MOTOR_DIR_PINS { 6, 10,  9,  8}   // 方向控制腳
#define MOTOR_PWM_PINS { 2,  5,  4,  3}   // PWM 控制腳
#define MOTOR_COUNT 4
#define MOTOR_SPEED_LIMIT 75.0f           // |speed| cap in percent (RCAP 2027)

#define SLP 11
#define BALL_POSESSION 38
#define START_BUTTON 27 // button 1, pressed = LOW (INPUT_PULLUP)

// Line flag from sense core: HIGH while a line is seen
#define PIN_0 0
#define PIN_1 1
#define LS_COUNT 16 // line sensors, sensor ch at ch * 22.5 deg, CCW, 0 = right, 90 = front


struct GyroData {
    float heading = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool exist = false;   // true only if a valid packet arrived this update
};

// Heading convention: 90 = facing forward (field frame), CCW positive
struct RobotState {
    float robot_heading = 90.0f;     // target heading used by Vector_Motion
    float heading_threshold = 2.0f;  // deg, no correction inside this band
    float P_factor = 0.5f;           // omega per degree of heading error
};

// Top camera (MaixCam, Serial3); angle/dist = 0xFFFF when goal not found
struct TopMaixData {
    bool valid = false;
    bool goal_found = false;
    uint16_t goal_angle = 0xFFFF; // deg 0-359
    uint16_t goal_dist = 0xFFFF;
};

extern GyroData gyroData;
extern TopMaixData topmaixData;
extern RobotState robot;
extern Adafruit_SSD1306 display;

void core_act_init();
void update_gyro_sensor();
void drawMessage(const char* msg);
void readTopMaix_polar();  // old protocol: send 0xDD, wait for reply (blocking)

// --- Top camera, new protocol: camera sends every frame, Serial3 RX by DMA ---
// After camera_dma_begin(), readTopMaix_polar() no longer works
void camera_dma_begin();
bool read_camera();        // non-blocking, true when topmaixData got a new packet

// --- Line (from sense core, Serial8) ---
bool line_flag();                       // true if PIN_0 or PIN_1 is HIGH
bool read_line_state(uint32_t& state);  // true if a valid packet arrived, keeps the newest
float line_angle(uint32_t state, float fallback = -1); // deg 0-360 of the line, -1 if none

// --- Motors ---
void SetMotorSpeed(uint8_t port, float speed);   // port 1-4, speed -100..100
void MotorStop();

// --- Motion ---
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, float rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
#endif