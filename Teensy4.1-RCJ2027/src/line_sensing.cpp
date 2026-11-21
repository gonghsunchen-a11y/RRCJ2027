#include <Arduino.h>
#include <EEPROM.h>
#include <core_sense.h>

// Line sensor raw test: mux 1 and mux 2 read in the same scan
// Mux 1: ring of 16 sensors, ch * 22.5 deg (0 = right, 90 = front)
// Mux 2: named sensors, edit M2_MAP if the wiring changes
//
// Scanning (button LINE_SCAN_BUTTON):
//   1st press: LED13 blinks, move the robot over white and black, min/max recorded
//   2nd press: LED13 off, threshold = (min + max) / 2, saved to EEPROM address 0
//   EEPROM layout same as core_sense: thr[0-15] = M1, thr[16-31] = M2
// L = below threshold (line seen), . = no line

#define BRIGHTNESS    105 // line sensor LED brightness (0-255)
#define MUX_SETTLE_US 10  // wait after selecting a channel
#define PRINT_MS      100 // print interval
#define LOW_DIFF      100 // max - min below this is flagged "<-- LOW" after a scan

// Mux 2 sensor table: name, channel
struct M2Sensor { const char* name; uint8_t ch; };
const M2Sensor M2_MAP[] = {
  {"F_out", 11}, {"F_in",  7},
  {"B_out",  5}, {"B_mid", 4}, {"B_in",  3},
  {"L_out",  2}, {"L_mid", 1}, {"L_in",  0},
  {"R_out", 15}, {"R_mid", 14}, {"R_in", 13},
};
const int M2_N = sizeof(M2_MAP) / sizeof(M2_MAP[0]);

uint16_t raw[32];          // M1 = 0-15, M2 = 16-31
uint16_t thr[32];          // thresholds, same index as raw
uint16_t mn[32], mx[32];   // min / max while scanning
bool scanning = false;

void setup() {
  Serial.begin(115200);
  pinMode(LS_S0, OUTPUT);
  pinMode(LS_S1, OUTPUT);
  pinMode(LS_S2, OUTPUT);
  pinMode(LS_S3, OUTPUT);
  pinMode(LS_M1, INPUT);
  pinMode(LS_M2, INPUT);
  pinMode(LS_BRIGHTNESS, OUTPUT);
  pinMode(LINE_SCAN_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, BRIGHTNESS);

  // Load saved thresholds, erased EEPROM reads 0xFFFF
  EEPROM.get(0, thr);
  for (int i = 0; i < 32; i++) {
    if (thr[i] == 0xFFFF) thr[i] = 512;
  }
}

// Mux 1 and mux 2 share S0-S3: one channel select, then read both
void readMux() {
  for (int ch = 0; ch < 16; ch++) {
    digitalWrite(LS_S0, (ch >> 0) & 1);
    digitalWrite(LS_S1, (ch >> 1) & 1);
    digitalWrite(LS_S2, (ch >> 2) & 1);
    digitalWrite(LS_S3, (ch >> 3) & 1);
    delayMicroseconds(MUX_SETTLE_US);
    raw[ch]      = analogRead(LS_M1);
    raw[ch + 16] = analogRead(LS_M2);
  }
}

// Button: 1st press start scan, 2nd press save
void checkButton() {
  static bool lastBtn = HIGH;
  static uint32_t lastPress = 0;

  bool btn = digitalRead(LINE_SCAN_BUTTON);
  if (lastBtn == HIGH && btn == LOW && millis() - lastPress > 200) { // 200 ms debounce
    lastPress = millis();
    scanning = !scanning;

    if (scanning) {
      for (int i = 0; i < 32; i++) { mn[i] = 0xFFFF; mx[i] = 0; }
      Serial.println("=== Scan start: move over white and black ===");
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      for (int i = 0; i < 32; i++) thr[i] = (mn[i] + mx[i]) / 2;
      EEPROM.put(0, thr);

      Serial.println("=== Scan saved ===");
      // diff = max - min, LOW marks a sensor that barely changed (did not see the line?)
      for (int ch = 0; ch < 16; ch++) {
        int d = mx[ch] - mn[ch];
        Serial.printf("M1 %6d deg  min %4d  max %4d  diff %4d  thr %4d%s\n",
                      (int)(ch * 22.5f), mn[ch], mx[ch], d, thr[ch], d < LOW_DIFF ? "  <-- LOW" : "");
      }
      for (int i = 0; i < M2_N; i++) {
        int k = M2_MAP[i].ch + 16;
        int d = mx[k] - mn[k];
        Serial.printf("M2 %6s ch%-2d min %4d  max %4d  diff %4d  thr %4d%s\n",
                      M2_MAP[i].name, M2_MAP[i].ch, mn[k], mx[k], d, thr[k], d < LOW_DIFF ? "  <-- LOW" : "");
      }
      Serial.println();
      delay(2000); // keep result on screen
    }
  }
  lastBtn = btn;
}

char mark(int i) { return raw[i] < thr[i] ? 'L' : '.'; }

void printData() {
  // Mux 1: angle, raw, line
  Serial.print(scanning ? "[SCAN]\n" : "");
  Serial.print("M1 deg :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", (int)(ch * 22.5f));
  Serial.print("\nM1 raw :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", raw[ch]);
  Serial.print("\nM1 line:");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6c", mark(ch));

  // Mux 2: name, channel, raw, line
  Serial.print("\nM2     :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6s", M2_MAP[i].name);
  Serial.print("\nM2 ch  :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6d", M2_MAP[i].ch);
  Serial.print("\nM2 raw :");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6d", raw[M2_MAP[i].ch + 16]);
  Serial.print("\nM2 line:");
  for (int i = 0; i < M2_N; i++) Serial.printf("%6c", mark(M2_MAP[i].ch + 16));
  Serial.print("\n\n");
}

void loop() {
  static uint32_t lastPrint = 0;

  readMux();
  checkButton();

  if (scanning) {
    for (int i = 0; i < 32; i++) {
      if (raw[i] < mn[i]) mn[i] = raw[i];
      if (raw[i] > mx[i]) mx[i] = raw[i];
    }
  }

  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    if (scanning) digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // blink while scanning
    printData();
  }
}