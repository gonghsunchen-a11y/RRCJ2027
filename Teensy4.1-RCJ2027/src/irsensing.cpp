#include <Arduino.h>
#include <ir_ball.h>
#include <math.h>

// Installed order: OUT 1/3 right, OUT 7 center, OUT 11/13 left.
// Nominal angles: 0 = right, 90 = front, 180 = left.
constexpr uint32_t PRINT_MS = 100;
constexpr int FIRST_FAR = 1;
constexpr int END_FAR = 4; // OUT 3, 7, and 11 are the installed far sensors.
uint16_t counts[IR_BALL_COUNT];

void setup() {
  Serial.begin(115200);
  Serial8.begin(BALL_SIDE_BAUD);
  initIrBallSensors();
}

void printBall(const char* label, int first, int end, int strongest) {
  const BallSide side = irPeakSide(counts, strongest);
  if (side == BallSide::None) {
    Serial.printf("%s NONE\n", label);
    return;
  }

  // Blend adjacent readings around the peak. The ends of this arc are not neighbours.
  float x = 0;
  float y = 0;
  for (int i = max(first, strongest - 1); i < min(end, strongest + 2); i++) {
    const float radians = IR_BALL_SENSORS[i].degrees * DEG_TO_RAD;
    x += counts[i] * cosf(radians);
    y += counts[i] * sinf(radians);
  }
  // Roundoff at the 180-degree endpoint can put y slightly below zero.
  const float angle = constrain(atan2f(max(0.0f, y), x) * RAD_TO_DEG, 0.0f, 180.0f);
  const auto& peak = IR_BALL_SENSORS[strongest];
  Serial.printf("%s angle %.1f peakSide=%s max out%u = %u\n", label, angle, ballSideName(side),
                static_cast<unsigned>(peak.out),
                static_cast<unsigned>(counts[strongest]));
}

void loop() {
  static uint32_t lastPrint = 0;

  readIrBallSensors(counts);
  const uint32_t now = millis();
  if (now - lastPrint < PRINT_MS) return;
  lastPrint = now;

  const int strongest = strongestIrSensor(counts, 0, IR_BALL_COUNT);
  sendIrBallSide(irPeakSide(counts, strongest));

  Serial.print("IR");
  for (int i = 0; i < IR_BALL_COUNT; i++) {
    Serial.printf(" out%u=%u", static_cast<unsigned>(IR_BALL_SENSORS[i].out),
                  static_cast<unsigned>(counts[i]));
  }
  Serial.println();
  printBall("Ball", 0, IR_BALL_COUNT, strongest);
  printBall("BallFar", FIRST_FAR, END_FAR, strongestIrSensor(counts, FIRST_FAR, END_FAR));
}
