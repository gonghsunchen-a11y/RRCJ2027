#include <Arduino.h>
#include <EEPROM.h>
#include <core_sense.h> // pin names only, no library code is called

// Mux 2 test: read + button calibration scan, written to merge into core_sense later
//  - S0-S3 are shared by both muxes, so one channel select reads M1 AND M2
//  - M2 uses bits 16-31 of the line state (same as the reserved code in core_sense)
//  - names are prefixed m2_ / static so nothing clashes with lsRaw, avg_ls, lineState
//  - M2 thresholds are saved at EEPROM address 32 = avg_ls[16..31] in core_sense,
//    so the M1 calibration (avg_ls[0..15], address 0-31) is never overwritten
//
// Calibration scan (same as calibrate_line_sensor, button on pin 36):
//  1st press: LED13 blinks, sweep all M2 sensors over green and white line
//  2nd press: threshold = (min + max) / 2, saved to EEPROM, result printed

// ===== Sensor layout: edit this table for a new PCB =====
// name, mux 2 channel (0-15)
struct LineSensor {
  const char* name;
  uint8_t ch;
};

static const LineSensor M2_SENSORS[] = {
  {"R_out",  0}, {"R_mid",  1}, {"R_in",  2},   // right
  {"L_out",  5}, {"L_mid",  4}, {"L_in",  3},   // left
  {"B_out",  8}, {"B_mid",  7}, {"B_in",  6},   // back
  {"F_out", 15},                {"F_in", 14},   // front
};
static const int M2_SENSOR_COUNT = sizeof(M2_SENSORS) / sizeof(M2_SENSORS[0]);

#define MUX_SETTLE_US  100 // shared by both muxes after each channel select
#define M2_EEPROM_ADDR 32  // avg_ls[16] in core_sense's EEPROM layout

static uint16_t m2_raw[16];            // M2 reading per mux channel
static uint16_t m1_raw[16];            // M1 reading, read in the same select (as after merging)
static uint16_t m2_thr[16];            // M2 threshold per mux channel
static uint32_t m2_state = 0xFFFFFFFF; // bit 16 + ch = 0 when below threshold 2 scans in a row

// One read of all 16 channels, both muxes (same structure as fast_update_line_sensor)
static void m2_update() {
  static uint32_t prevRaw = 0xFFFFFFFF;
  uint32_t rawState       = 0xFFFFFFFF;

  for (uint8_t ch = 0; ch < 16; ch++) {
    digitalWriteFast(LS_S0, (ch >> 0) & 1);
    digitalWriteFast(LS_S1, (ch >> 1) & 1);
    digitalWriteFast(LS_S2, (ch >> 2) & 1);
    digitalWriteFast(LS_S3, (ch >> 3) & 1);
    delayMicroseconds(MUX_SETTLE_US);

    m1_raw[ch] = analogRead(LS_M1);
    m2_raw[ch] = analogRead(LS_M2);

    if (m2_raw[ch] < m2_thr[ch]) rawState &= ~(1UL << (ch + 16));
  }

  // Bit clears only if below threshold in this scan AND the previous one
  m2_state = prevRaw | rawState | 0x0000FFFF; // M1 bits not used in this test
  prevRaw  = rawState;
}

// Call every loop after m2_update(). Returns true while scanning
static bool m2_calibrate() {
  static bool scanning = false;
  static bool lastBtn = HIGH;
  static uint32_t lastPress = 0;
  static uint32_t blinkTimer = 0;
  static uint16_t maxV[16], minV[16];

  bool btn = digitalRead(LINE_SCAN_BUTTON);
  if (lastBtn == HIGH && btn == LOW && millis() - lastPress > 200) { // pressed, 200 ms debounce
    lastPress = millis();
    scanning = !scanning;

    if (scanning) {
      for (int ch = 0; ch < 16; ch++) {
        maxV[ch] = 0;
        minV[ch] = 0xFFFF;
      }
      Serial.println("M2 scan start");
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      for (int i = 0; i < M2_SENSOR_COUNT; i++) {
        uint8_t ch = M2_SENSORS[i].ch;
        m2_thr[ch] = (maxV[ch] + minV[ch]) / 2;
        Serial.printf("%-6s ch%2d: min=%d, max=%d, thr=%d\n",
                      M2_SENSORS[i].name, ch, minV[ch], maxV[ch], m2_thr[ch]);
      }
      EEPROM.put(M2_EEPROM_ADDR, m2_thr);
      Serial.println("M2 scan saved");
      delay(2000); // keep result on screen
    }
  }
  lastBtn = btn;

  if (scanning) {
    for (int ch = 0; ch < 16; ch++) {
      if (m2_raw[ch] > maxV[ch]) maxV[ch] = m2_raw[ch];
      if (m2_raw[ch] < minV[ch]) minV[ch] = m2_raw[ch];
    }
    if (millis() - blinkTimer > 100) {
      blinkTimer = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
  return scanning;
}

void setup() {
  Serial.begin(115200);
  pinMode(LS_S0, OUTPUT);
  pinMode(LS_S1, OUTPUT);
  pinMode(LS_S2, OUTPUT);
  pinMode(LS_S3, OUTPUT);
  pinMode(LS_M1, INPUT);
  pinMode(LS_M2, INPUT);
  pinMode(LINE_SCAN_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LS_BRIGHTNESS, OUTPUT);
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 255); // line sensor LED brightness (0-255)

  // Load M2 thresholds; erased EEPROM reads 0xFFFF
  EEPROM.get(M2_EEPROM_ADDR, m2_thr);
  if (m2_thr[0] == 0xFFFF) {
    for (int ch = 0; ch < 16; ch++) m2_thr[ch] = 512;
  }
}

void loop() {
  m2_update();
  bool scanning = m2_calibrate();

  // Print every 100 ms: raw value per sensor, plus L when it sees the line
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  static int lines = 0;
  if (lines++ % 20 == 0) {
    for (int i = 0; i < M2_SENSOR_COUNT; i++) Serial.printf("%7s ", M2_SENSORS[i].name);
    Serial.println();
  }

  if (scanning) Serial.print("SCAN ");
  for (int i = 0; i < M2_SENSOR_COUNT; i++) {
    uint8_t ch = M2_SENSORS[i].ch;
    bool line = !((m2_state >> (ch + 16)) & 1);
    Serial.printf("%5d %c ", m2_raw[ch], line ? 'L' : '-');
  }
  Serial.println();
}
