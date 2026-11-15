#include <Arduino.h>
#include <core_act.h>

// Defense, act core
// Front inner on the line -> move backward
// Front outer on the line -> move forward
// Sideways: default speed, but slow down and stop at a side line:
//   out sensor on the line -> slow,  mid -> stop,  in -> back off
// The side limits only cap speed toward that line, never push the robot into it

#define SPEED       30 // forward / backward
#define SIDE_SPEED  30 // default sideways speed (+ = right)
#define SLOW_SPEED  10 // out sensor sees the side line
#define BACK_SPEED  15 // in sensor sees the side line

// Sensors as lineState bits (mux 2 channel + 16)
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(14)
#define FRONT_OUT M2(15)
#define R_OUT M2(0)
#define R_MID M2(1)
#define R_IN  M2(2)
#define L_OUT M2(5)
#define L_MID M2(4)
#define L_IN  M2(3)

void setup() {
  core_act_init();

  // Wait for button 1, then run until restart
  MotorStop();
  drawMessage("Press BTN");
  while (digitalRead(START_BUTTON) == HIGH) update_gyro_sensor();
  delay(200); // debounce
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
  return SIDE_SPEED;                        // no line: full speed
}

void loop() {
  static uint32_t state = 0xFFFFFFFF; // bit = 0 when that sensor sees the line

  update_gyro_sensor();
  read_line_state(state);                // newest packet from sense core
  if (!line_flag()) state = 0xFFFFFFFF;  // flag LOW = no line anywhere

  // Forward / backward from the front sensors
  bool in  = seen(state, FRONT_IN);
  bool out = seen(state, FRONT_OUT);
  float vy = 0;
  if (in && !out)      vy = -SPEED; // backward
  else if (out && !in) vy =  SPEED; // forward

  // Sideways: default speed, capped by each side line
  float vx = SIDE_SPEED;
  vx = min(vx,  sideLimit(state, R_OUT, R_MID, R_IN)); // right line
  vx = max(vx, -sideLimit(state, L_OUT, L_MID, L_IN)); // left line

  Vector_Motion(vx, vy, 0);
}
