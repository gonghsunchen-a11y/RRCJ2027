#pragma once

#include <Arduino.h>
#include <ball_side.h>

struct IrSensor {
  uint8_t out;
  uint8_t pin;
  int16_t degrees;
};

constexpr IrSensor IR_BALL_SENSORS[] = {
  {1, 12, 0}, {3, 10, 45}, {7, 6, 90}, {11, 2, 135}, {13, 22, 180},
};
constexpr int IR_BALL_COUNT = sizeof(IR_BALL_SENSORS) / sizeof(IR_BALL_SENSORS[0]);

inline void initIrBallSensors() {
  for (const auto& sensor : IR_BALL_SENSORS) pinMode(sensor.pin, INPUT);
}

inline void readIrBallSensors(uint16_t (&counts)[IR_BALL_COUNT]) {
  for (auto& count : counts) count = 0;

  const uint32_t start = micros();
  while (micros() - start < 1000) {
    for (int i = 0; i < IR_BALL_COUNT; i++) {
      if (digitalRead(IR_BALL_SENSORS[i].pin) == LOW) counts[i]++;
    }
  }
}

inline int strongestIrSensor(const uint16_t (&counts)[IR_BALL_COUNT], int first, int end) {
  int strongest = first;
  for (int i = first + 1; i < end; i++) {
    if (counts[i] > counts[strongest]) strongest = i;
  }
  return strongest;
}

inline BallSide irPeakSide(const uint16_t (&counts)[IR_BALL_COUNT], int strongest) {
  if (counts[strongest] == 0) return BallSide::None;
  const int angle = IR_BALL_SENSORS[strongest].degrees;
  if (angle < 90) return BallSide::Right;
  if (angle > 90) return BallSide::Left;
  return BallSide::Center;
}

inline void sendIrBallSide(BallSide side) {
  const uint8_t payload = static_cast<uint8_t>(side);
  const uint8_t packet[] = {BALL_SIDE_HEADER, payload, ballSideChecksum(payload)};
  Serial8.write(packet, sizeof(packet));
}
