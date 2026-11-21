#include <Arduino.h>
#include <core_act.h>

void setup() {
  Serial.begin(115200);
  Serial8.begin(BALL_SIDE_BAUD);
}

void loop() {
  static BallSide side = BallSide::None;
  static uint32_t lastPacket = 0;
  static bool received = false;
  static bool reportedTimeout = false;

  static uint32_t lineState = 0xFFFFFFFF;
  if (read_sense_state(lineState, side).ball) {
    received = true;
    reportedTimeout = false;
    lastPacket = millis();
    Serial.printf("Ball side=%s\n", ballSideName(side));
  } else if ((!received || millis() - lastPacket > 500) && !reportedTimeout) {
    reportedTimeout = true;
    Serial.println("Ball side=NO_DATA");
  }
}
