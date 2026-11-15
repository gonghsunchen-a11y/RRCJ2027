#include <Arduino.h>
#include <core_act.h>



// Defense, act core
// Front inner on the line -> move backward
// Front outer on the line -> move forward
// Sideways: default speed, but slow down and stop at a side line:
//   out sensor on the line -> slow,  mid -> stop,  in -> back off
// The side limits only cap speed toward that line, never push the robot into it

#define SPEED       30 // forward / backward
#define SIDE_SPEED  40 // default sideways speed (+ = right, - = left)
#define SLOW_SPEED  10 // out sensor sees the side line
#define BACK_SPEED  15 // in sensor sees the side line

// Sensors as lineState bits (mux 2 channel + 16)
#define M1(ch) (1UL << (ch))
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(14)
#define FRONT_OUT M2(15)
#define R_OUT M2(0)
#define R_MID M2(1)
#define R_IN  M2(2)
#define L_OUT M2(5)
#define L_MID M2(4)
#define L_IN  M2(3)

#define midls M1(4)

static bool cameraReceived = false;
static uint32_t lastCameraPacket = 0;

void setup() {
  core_act_init();
  camera_dma_begin();

  MotorStop();

}

// true if any sensor in mask sees the line (bit = 0)
bool seen(uint32_t state, uint32_t mask) {
  return (~state & mask) != 0;
}

// Max speed toward a side line, from how far the robot is over it
float sideLimit(uint32_t state, uint32_t out, uint32_t mid, uint32_t in) {
  if (seen(state, in))  return -BACK_SPEED; // too far: back off
  if (seen(state, mid)) return 0;           // on the line: stop
  if (seen(state, out)) return SLOW_SPEED;  // line reached: slow down
  return fabsf((float)SIDE_SPEED);           // positive limit for either direction
}

void defense() {

  if (read_camera()) {
    cameraReceived = true;
    lastCameraPacket = millis();
  }
  static uint32_t state = 0xFFFFFFFF; // bit = 0 when that sensor sees the line

  update_gyro_sensor();
  read_line_state(state);                // newest packet from sense core
  if (!line_flag()) state = 0xFFFFFFFF;  // flag LOW = no line anywhere

  // Forward / backward from the front sensors
  bool in  = seen(state, FRONT_IN);
  bool out = seen(state, FRONT_OUT);
  bool mid = seen(state, midls);
  static bool prevIn = false, prevMid = false;
  static bool rearLeftFirst = false, recovering = false;
  static uint32_t recoveryStarted = 0;
  if (recovering && mid) {
    recovering = false;
  } else if (!recovering) {
    if (prevIn && !in && prevMid && mid) rearLeftFirst = true;
    if (in) rearLeftFirst = false;
    if (rearLeftFirst && prevMid && !mid) {
      recovering = true;
      rearLeftFirst = false;
      recoveryStarted = millis();
    }
  }
  prevIn = in;
  prevMid = mid;
  float vy = 0;
  if (!mid) {
    if (in && !out) vy = -SPEED;
    else if (out && !in) vy = SPEED;
  }

  // Sideways: default speed, capped by each side line
  float vx = SIDE_SPEED;
  // Camera caps normal sideways travel; line backoff below retains priority.
  if (cameraReceived && millis() - lastCameraPacket <= 500 &&
      topmaixData.valid && topmaixData.goal_found && topmaixData.goal_angle < 360) {
    float angle = topmaixData.goal_angle;
    float speedCap = 40.0f;
    if (vx > 0) {
      // Right: 280 deg = 40, 270 deg = 10, 265 deg or below = stopped.
      if (angle <= 265.0f) speedCap = 0;
      else if (angle < 270.0f) speedCap = 2.0f * (angle - 265.0f);
      else if (angle < 280.0f) speedCap = 10.0f + 3.0f * (angle - 270.0f);
    } else if (vx < 0) {
      // Left: full speed through 275 deg, linearly stop at 300 deg.
      speedCap = constrain(40.0f * (300.0f - angle) / 25.0f, 0.0f, 40.0f);
    }
    vx = constrain(vx, -speedCap, speedCap);
  }

  vx = min(vx,  sideLimit(state, R_OUT, R_MID, R_IN)); // right line
  vx = max(vx, -sideLimit(state, L_OUT, L_MID, L_IN)); // left line

  // Latch corner contact: hold the line even if contact or heading changes.
  static bool rightLocked = false, leftLocked = false;
  bool cornerEnabled = gyroData.heading >= -10.0f && gyroData.heading <= 10.0f && (in || out);
  if (cornerEnabled && seen(state, R_OUT | R_MID | R_IN)) rightLocked = true;
  if (cornerEnabled && seen(state, L_OUT | L_MID | L_IN)) leftLocked = true;
  if (rightLocked || leftLocked) {
    vx = 0;
    vy = 0;
    // Correct only while an inner sensor confirms overshoot; stop as soon as it clears.
    if (rightLocked && !leftLocked && seen(state, R_IN)) vx = -BACK_SPEED;
    if (leftLocked && !rightLocked && seen(state, L_IN)) vx = BACK_SPEED;

  }

  if (recovering) {
    vx = 0;
    vy = min(60.0f, SPEED + (millis() - recoveryStarted) * 0.01f);
  }
  if ((rightLocked || leftLocked) && vx == 0 && vy == 0) MotorStop();
  else Vector_Motion(vx, vy, 0);
}

static int returnStage = 0;
void backcourtfunction(){
    //MotorStop();
    read_camera();
    update_gyro_sensor();

    if (!topmaixData.valid || !topmaixData.goal_found) {
        MotorStop();
        return;
    } else {
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
                15.0f + (abs(angleError) - 15) * 0.1f,
                15.0f, 20.0f
            );
        } else {
            // 中間：越接近 distance 28 越慢
            speed = constrain(
                15.0f + abs(distance - 25) * 0.2f,
                15.0f, 20.0f
            );
        }

        if (returnStage == 2) {
            if (abs(angleError) > 5) {
                // 此階段只左右移動，依角度減速
                float sideSpeed = constrain(
                    15.0f + (abs(angleError) - 5) * 0.1f,
                    15.0f, 20.0f
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

void loop() {
}
