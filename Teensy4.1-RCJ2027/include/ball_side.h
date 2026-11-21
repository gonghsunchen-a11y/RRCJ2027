#pragma once

#include <stdint.h>

enum class BallSide : uint8_t { None, Right, Center, Left };

// Serial8 packet: header, side, checksum. Line-sensor packets use header 0xAA.
constexpr uint32_t BALL_SIDE_BAUD = 921600;
constexpr uint8_t BALL_SIDE_HEADER = 0xB7;

inline uint8_t ballSideChecksum(uint8_t side) {
  return static_cast<uint8_t>(~(BALL_SIDE_HEADER ^ side));
}

inline const char* ballSideName(BallSide side) {
  switch (side) {
    case BallSide::Right:  return "RIGHT";
    case BallSide::Center: return "CENTER";
    case BallSide::Left:   return "LEFT";
    case BallSide::None:   return "NONE";
  }
  return "NONE";
}
