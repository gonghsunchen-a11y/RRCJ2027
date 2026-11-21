#include <Arduino.h>
#include <core_sense.h>
#include <ir_ball.h>

// Defense, sense core
// Line seen (mux 1 or mux 2): PIN_0/PIN_1 HIGH, then lineState (32 bits) to act core
//   bit 0-15  = mux 1 channel 0-15
//   bit 16-31 = mux 2 channel 0-15

#define M2_UNUSED (0x1740UL << 16) // mux 2 channel 6, 8, 9, 10, 12 have no sensor
#define M2_IGNORE (0x8024UL << 16) // R_out 15, B_out 5, L_out 2: read but never reported
#define DEBUG 1                    // 1 = print sensor data every 100 ms

// Mux 2 sensor names for the debug print, same as line_sensing.cpp
struct M2Sensor { const char* name; uint8_t ch; };
const M2Sensor M2_MAP[] = {
  {"F_out", 11}, {"F_in",  7},
  {"B_out",  5}, {"B_mid", 4}, {"B_in",  3},
  {"L_out",  2}, {"L_mid", 1}, {"L_in",  0},
  {"R_out", 15}, {"R_mid", 14}, {"R_in", 13},
};
const int M2_N = sizeof(M2_MAP) / sizeof(M2_MAP[0]);

// L = line seen (bit = 0), . = no line
char mark(int i) { return (lineState >> i) & 1 ? '.' : 'L'; }

// Debug print: mux 1 by angle, mux 2 by name. raw, threshold, line
void printDebug() {
  Serial.printf("flag %d  state %08lX\n", lineState != 0xFFFFFFFF, lineState);

  Serial.print("M1 deg :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", (int)(ch * 22.5f));
  Serial.print("\nM1 raw :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", lsRaw[ch]);
  Serial.print("\nM1 thr :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", avg_ls[ch]);
  Serial.print("\nM1 line:");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6c", mark(ch));

  Serial.print("\nM2     :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6s", M2_MAP[i].name);
  Serial.print("\nM2 ch  :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6d", M2_MAP[i].ch);
  Serial.print("\nM2 raw :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6d", lsRaw[M2_MAP[i].ch + 16]);
  Serial.print("\nM2 thr :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6d", avg_ls[M2_MAP[i].ch + 16]);
  Serial.print("\nM2 line:");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6c", mark(M2_MAP[i].ch + 16));
  Serial.print("\n\n");
}

void setup() {
  core_sense_init();
  initIrBallSensors();
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 105); // same as line_sensing.cpp calibration (0-255)
}

void loop() {
  fast_update_line_sensor();
  lineState |= M2_UNUSED | M2_IGNORE; // never report a line on these channels

  if (!calibrate_line_sensor()) { // button pin 36 calibrates mux 1
    send_line_state();

    static uint32_t lastBall = 0;
    const uint32_t now = millis();
    if (now - lastBall >= 20) {
      lastBall = now;
      uint16_t counts[IR_BALL_COUNT];
      readIrBallSensors(counts);
      sendIrBallSide(irPeakSide(counts, strongestIrSensor(counts, 0, IR_BALL_COUNT)));
    }
    digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF); // LED 13 on = line seen

    static uint32_t lastPrint = 0;
    if (DEBUG && millis() - lastPrint >= 100) {
      lastPrint = millis();
      printDebug();
    }
  }
}
