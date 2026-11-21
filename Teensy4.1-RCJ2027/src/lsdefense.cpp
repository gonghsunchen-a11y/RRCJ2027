#include <Arduino.h>
#include <core_act.h>

// Defense, act core
// Front middle on the line -> hold depth
// If middle is clear: front inner only -> backward, front outer only -> forward
// If pushed back off the front line -> recover forward until middle sees it
// Outer sensors R_out / B_out / L_out are ignored (sense core never reports them)
// Rear mid/in: one can request reverse, both stop. The ring can latch reverse.
// Sideways: default speed, stop with the MID sensor on the side line
//   mid on the line -> stop here
//   in on the line  -> back off until mid sees the line again
// The side limits only cap speed toward that line, never push the robot into it
// Normal defense blocks backward motion at a side line. Line recovery takes priority.
// Camera brakes sideways travel by goal angle before the side-line limits.

#define SPEED       15  // forward / backward
#define SIDE_SPEED  50  // requested sideways speed before braking
#define BACK_SPEED  15  // backing off from a side line
#define LOST_MS     300 // stop backing if that side sees no line this long

#define GOAL_TIMEOUT_MS 500
constexpr uint32_t BALL_TIMEOUT_MS = 200;
constexpr float MIN_VX = 15.0f;
constexpr int BACKCOURT_TARGET_ANGLE = 270;
constexpr int BACKCOURT_ALIGN_ERROR = 330 - BACKCOURT_TARGET_ANGLE; // 210–330 degrees
constexpr int BACKCOURT_ANGLE_TOLERANCE = 5;
constexpr float BACKCOURT_MIN_SPEED = 20.0f;
constexpr float BACKCOURT_MAX_SPEED = 30.0f;
constexpr float BACKCOURT_REVERSE_SPEED = 35.0f;
constexpr int BACKCOURT_MIN_DISTANCE = 25;
constexpr int BACKCOURT_MAX_DISTANCE = 45;
constexpr int BACKCOURT_FAR_ENTRY_DISTANCE = 70;

enum class BackcourtStage { Approach, Forward, Align };
enum class PushInStage { Idle, WaitForFront, Recover };
constexpr float MIN_PUSH_IN_SPEED = 20.0f;
constexpr float MAX_PUSH_IN_SPEED = 50.0f;
constexpr float PUSH_IN_ACCELERATION = 0.02f; // Speed units per millisecond

// Sensors as lineState bits (mux 2 channel + 16)
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(7)
#define FRONT_OUT M2(11)
#define FRONT_MID (1UL << 4) // Mux 1 channel 4 (90 degrees)
constexpr uint32_t RIGHT_RING = 1UL << 0;
constexpr uint32_t LEFT_RING = 1UL << 8;
constexpr uint32_t SIDE_RING = RIGHT_RING | LEFT_RING;
constexpr uint32_t REAR_RING = 0xFE00UL; // Mux 1 ch9–15
constexpr float FAR_RING_REVERSE_SPEED = 30.0f;
#define BACK  (M2(4) | M2(3)) // mid, in (B_out ignored)

// One side line: its sensors and backing-off memory
struct Side {
  uint32_t mid, in;
  bool backing;
  uint32_t lastSeen;
};
Side rightSide = {M2(14), M2(13), false, 0}; // mid, in (R_out ignored)
Side leftSide  = {M2(1),  M2(0),  false, 0}; // mid, in (L_out ignored)

// true if any sensor in mask sees the line (bit = 0)
bool seen(uint32_t state, uint32_t mask) {
  return (~state & mask) != 0;
}

// Max speed toward a side line (negative = move away from it)
float sideLimit(Side& s, uint32_t state) {
  bool mid = seen(state, s.mid);
  bool in  = seen(state, s.in);

  if (mid || in) s.lastSeen = millis();
  if (in) s.backing = true;                               // too far over the line
  if (mid && !in) s.backing = false;                      // back at the mid sensor
  if (millis() - s.lastSeen > LOST_MS) s.backing = false; // line lost, give up

  if (s.backing) return -BACK_SPEED; // back off until mid sees the line
  if (mid)       return 0;           // mid on the line: stay here
  return 1000;                       // no line: no limit
}

// Debug screen while waiting for the button. X = sensor sees the line
char mark(uint32_t state, uint32_t mask) { return seen(state, mask) ? 'X' : '.'; }

void drawDebug(uint32_t state) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Press BTN to start");
  display.printf("Head  %.1f\n", gyroData.heading);
  if (topmaixData.goal_found) display.printf("Goal  A%d D%d\n", topmaixData.goal_angle, topmaixData.goal_dist);
  else                        display.println("Goal  none");
  display.printf("Flag  %d\n", line_flag());
  display.printf("Front i%c m%c o%c\n", mark(state, FRONT_IN),
                 mark(state, FRONT_MID), mark(state, FRONT_OUT));
  display.printf("Left  m%c i%c\n", mark(state, leftSide.mid), mark(state, leftSide.in));
  display.printf("Right m%c i%c\n", mark(state, rightSide.mid), mark(state, rightSide.in));
  display.printf("Back  m%c i%c\n", mark(state, M2(4)), mark(state, M2(3)));
  display.display();
}

void setup() {
  core_act_init();
  camera_dma_begin();

  // Wait for button 1 showing live sensor data, then run until restart
  MotorStop();
  uint32_t state = 0xFFFFFFFF, lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera();
    read_line_state(state);
    if (!line_flag()) state = 0xFFFFFFFF;
    if (millis() - lastDraw >= 100) { // OLED is slow, redraw every 100 ms
      lastDraw = millis();
      drawDebug(state);
    }
  }
  drawMessage("GO");
  delay(200); // debounce
}

struct MotionStatus {
  uint32_t state;
  bool onLine;
  bool goalAvailable;
  BallSide ballSide;

  void print(const char* mode, float vx, float vy) const {
    static uint32_t lastPrint = 0;
    const uint32_t now = millis();
    if (now - lastPrint < 200) return;
    lastPrint = now;

    Serial.printf("mode=%s vx=%.1f vy=%.1f ball=%s goalAngle=%u goalDist=%u cam=%d line=%d "
                  "R(out/mid/in/ring)=%d/%d/%d/%d L(out/mid/in/ring)=%d/%d/%d/%d "
                  "F(out/mid/in)=%d/%d/%d B(mid/in)=%d/%d rearRing=%d\n",
                  mode, vx, vy, ballSideName(ballSide), static_cast<unsigned>(topmaixData.goal_angle),
                  static_cast<unsigned>(topmaixData.goal_dist), goalAvailable, onLine,
                  seen(state, M2(15)), seen(state, rightSide.mid), seen(state, rightSide.in),
                  seen(state, RIGHT_RING), seen(state, M2(2)), seen(state, leftSide.mid),
                  seen(state, leftSide.in), seen(state, LEFT_RING), seen(state, FRONT_OUT),
                  seen(state, FRONT_MID), seen(state, FRONT_IN), seen(state, M2(4)),
                  seen(state, M2(3)), seen(state, REAR_RING));
  }

  void command(const char* mode, float vx, float vy) const {
    if (vx != 0 && fabsf(vx) < MIN_VX) vx = copysignf(MIN_VX, vx);
    Vector_Motion(vx, vy, 0);
    print(mode, vx, vy);
  }
};

void runBackcourt(int angleError, int distance, BackcourtStage& stage, const MotionStatus& motion) {
  const int error = abs(angleError);
  if (stage == BackcourtStage::Approach && error > BACKCOURT_ANGLE_TOLERANCE) {
    stage = error > BACKCOURT_ALIGN_ERROR ? BackcourtStage::Forward : BackcourtStage::Align;
  }
  if (stage == BackcourtStage::Forward) {
    if (error > BACKCOURT_ALIGN_ERROR) {
      motion.command("BACKCOURT_FORWARD", 0, BACKCOURT_MAX_SPEED);
      return;
    }
    stage = BackcourtStage::Align;
  }

  if (stage == BackcourtStage::Align) {
    if (error > BACKCOURT_ANGLE_TOLERANCE) {
      const float sideSpeed = constrain(
          BACKCOURT_MIN_SPEED + (error - BACKCOURT_ANGLE_TOLERANCE) * 0.1f,
          BACKCOURT_MIN_SPEED, BACKCOURT_MAX_SPEED);
      motion.command("BACKCOURT_ALIGN", angleError > 0 ? sideSpeed : -sideSpeed, 0);
      return;
    }
    stage = BackcourtStage::Approach;
  }

  const float speed = error > 12
      ? constrain(BACKCOURT_MIN_SPEED + (error - 15) * 0.1f, BACKCOURT_MIN_SPEED, BACKCOURT_MAX_SPEED)
      : constrain(BACKCOURT_MIN_SPEED + abs(distance - BACKCOURT_MIN_DISTANCE) * 0.2f,
                  BACKCOURT_MIN_SPEED, BACKCOURT_MAX_SPEED);
  float vx = 0;
  float vy = 0;
  if (error > BACKCOURT_ANGLE_TOLERANCE) vx = angleError > 0 ? speed : -speed;
  if (distance < BACKCOURT_MIN_DISTANCE) vy = speed;
  else if (distance > BACKCOURT_MAX_DISTANCE) vy = -BACKCOURT_REVERSE_SPEED;

  if (vx != 0 && vy != 0) {
    vx *= 0.7071f;
    vy *= 0.7071f;
  }
  motion.command("BACKCOURT_APPROACH", vx, vy);
}

void loop() {
  static uint32_t state = 0xFFFFFFFF; // bit = 0 when that sensor sees the line

  static bool cameraReceived = false;
  static uint32_t lastCameraPacket = 0;
  update_gyro_sensor();
  if (read_camera()) {
    cameraReceived = true;
    lastCameraPacket = millis();
  }
  static BallSide ballSide = BallSide::None;
  static uint32_t lastBallPacket = 0;
  if (read_sense_state(state, ballSide).ball) lastBallPacket = millis();
  if (millis() - lastBallPacket > BALL_TIMEOUT_MS) ballSide = BallSide::None;
  const bool onLine = line_flag();
  if (!onLine) state = 0xFFFFFFFF;

  const bool goalAvailable = cameraReceived && millis() - lastCameraPacket <= GOAL_TIMEOUT_MS &&
                             topmaixData.valid && topmaixData.goal_found &&
                             topmaixData.goal_angle < 360;
  const MotionStatus motion{state, onLine, goalAvailable, ballSide};
  const int angle = topmaixData.goal_angle;
  const int distance = topmaixData.goal_dist;
  int angleError = angle - BACKCOURT_TARGET_ANGLE;
  if (angleError < -180) angleError += 360;
  const bool positionKnown = goalAvailable && distance != 0xFFFF;
  const bool backcourtRequested = abs(angleError) > BACKCOURT_ALIGN_ERROR ||
                                  distance > BACKCOURT_FAR_ENTRY_DISTANCE;
  const bool outsideTarget = abs(angleError) > BACKCOURT_ANGLE_TOLERANCE ||
                             distance < BACKCOURT_MIN_DISTANCE || distance > BACKCOURT_MAX_DISTANCE;

  const bool in = seen(state, FRONT_IN);
  const bool mid = seen(state, FRONT_MID);
  const bool out = seen(state, FRONT_OUT);
  static bool prevIn = false, prevMid = false;
  static bool innerLeftFirst = false, recovering = false;

  static PushInStage pushIn = PushInStage::Idle;
  static bool lastFrontMid = false;
  static uint32_t pushInStarted = 0;
  if (mid) {
    pushIn = PushInStage::Idle;
  } else if (pushIn == PushInStage::Idle && lastFrontMid && out) {
    pushIn = PushInStage::WaitForFront;
  } else if (pushIn == PushInStage::WaitForFront && !out) {
    pushIn = PushInStage::Recover;
    pushInStarted = millis();
  }
  lastFrontMid = mid;

  const bool enterFromRight = goalAvailable &&
      angle < BACKCOURT_TARGET_ANGLE - BACKCOURT_ALIGN_ERROR &&
      seen(state, leftSide.mid | leftSide.in | LEFT_RING);
  const bool enterFromLeft = goalAvailable &&
      angle > BACKCOURT_TARGET_ANGLE + BACKCOURT_ALIGN_ERROR &&
      seen(state, rightSide.mid | rightSide.in | RIGHT_RING);

  const bool pushInDetected = pushIn != PushInStage::Idle;
  static bool reverseRecovering = false;
  reverseRecovering = !mid && !pushInDetected &&
                   (reverseRecovering || seen(state, SIDE_RING | REAR_RING));

  static bool backcourtActive = false;
  static BackcourtStage backcourtStage = BackcourtStage::Approach;
  // Once vision confirms we are out of position off the line, Backcourt owns the return.
  if (!onLine && positionKnown && outsideTarget &&
      (backcourtActive || backcourtRequested)) {
    reverseRecovering = false;
    pushIn = PushInStage::Idle;
    if (!backcourtActive) {
      backcourtActive = true;
      prevIn = prevMid = innerLeftFirst = recovering = false;
      leftSide.backing = rightSide.backing = false;
      Serial.println("BACKCOURT");
    }
    runBackcourt(angleError, distance, backcourtStage, motion);
    return;
  }
  if (!onLine && backcourtActive && !positionKnown) {
    MotorStop();
    motion.print("BACKCOURT_WAIT_CAMERA", 0, 0);
    return;
  }
  if (backcourtActive) {
    backcourtActive = false;
    backcourtStage = BackcourtStage::Approach;
    Serial.println("DEFENSE");
  }

  // The touched side is inside the field. Cross the boundary toward that side.
  if (enterFromRight || enterFromLeft) {
    pushIn = PushInStage::Idle;
    reverseRecovering = false;
    prevIn = prevMid = innerLeftFirst = recovering = false;
    leftSide.backing = rightSide.backing = false;
    motion.command(enterFromRight ? "ENTER_FROM_RIGHT" : "ENTER_FROM_LEFT",
                   enterFromRight ? -BACK_SPEED : BACK_SPEED, 0);
    return;
  }

  if (pushInDetected) {
    prevIn = prevMid = innerLeftFirst = recovering = false;
    leftSide.backing = rightSide.backing = false;
    if (pushIn == PushInStage::WaitForFront) {
      motion.command("PUSH_IN_FRONT_CONTACT", 0, MIN_PUSH_IN_SPEED);
    } else {
      const float vy = min(MAX_PUSH_IN_SPEED,
                           MIN_PUSH_IN_SPEED + (millis() - pushInStarted) * PUSH_IN_ACCELERATION);
      motion.command("PUSH_IN_RECOVERY", 0, vy);
    }
    return;
  }

  if (reverseRecovering) {
    prevIn = prevMid = innerLeftFirst = recovering = false;
    leftSide.backing = rightSide.backing = false;
    const float reverseSpeed = seen(state, REAR_RING) ? FAR_RING_REVERSE_SPEED : SPEED;
    motion.command("RING_REVERSE", 0, -reverseSpeed);
    return;
  }

  if (recovering && mid) {
    recovering = false;
  } else if (!recovering) {
    if (prevIn && !in && prevMid && mid) innerLeftFirst = true;
    if (in) innerLeftFirst = false;
    if (innerLeftFirst && prevMid && !mid) {
      recovering = true;
      innerLeftFirst = false;
    }
  }
  prevIn = in;
  prevMid = mid;

  const int back = __builtin_popcount(~state & BACK);
  if (back == 2) {
    motion.command("REAR_STOP", 0, 0);
    return;
  }

  float vy = 0;
  if (!mid) {
    if (in && !out)      vy = -SPEED;
    else if (out && !in) vy = MIN_PUSH_IN_SPEED;
    else if (back > 0)   vy = -SPEED;
  }

  // At a side line the curve also reaches the front sensors: never move backward there
  bool atSide = seen(state, rightSide.mid | rightSide.in | leftSide.mid | leftSide.in) ||
                rightSide.backing || leftSide.backing;
  if (atSide) vy = max(vy, 0.0f);

  float vx = 0;
  if (ballSide == BallSide::Right) vx = abs(SIDE_SPEED);
  else if (ballSide == BallSide::Left) vx = -abs(SIDE_SPEED);
  // Camera caps normal sideways travel; line backoff below retains priority.
  if (goalAvailable) {
    float speedCap = 40.0f;
    if (vx > 0) {
      // Right: stop at 240 degrees, with a minimum speed before stopping.
      if (angle <= 240.0f) speedCap = 0;
      else if (angle < 280.0f) {
        speedCap = max(MIN_VX, 10.0f + 3.0f * (angle - 270.0f));
      }
    } else if (vx < 0) {
      // Left: stop at 300 degrees, with the same minimum speed before stopping.
      if (angle >= 300.0f) speedCap = 0;
      else speedCap = constrain(40.0f * (300.0f - angle) / 25.0f, MIN_VX, 40.0f);
    }
    vx = constrain(vx, -speedCap, speedCap);
  }

  float rightLim = sideLimit(rightSide, state);
  float leftLim = sideLimit(leftSide, state);
  vx = max(min(vx, rightLim), -leftLim);

  if (recovering) {
    vx = 0;
    vy = SPEED;
  }

  if (!recovering && (ballSide == BallSide::None ||
                      (ballSide == BallSide::Center && vx == 0 && vy == 0))) {
    MotorStop();
    motion.print(ballSide == BallSide::None ? "NO_BALL" : "DEFENSE", 0, 0);
    return;
  }

  const char* mode = "DEFENSE";
  if (recovering) mode = "FRONT_RECOVERY";
  else if (vy < 0) mode = in && !out ? "FRONT_REVERSE" : "REAR_REVERSE";
  motion.command(mode, vx, vy);
}
