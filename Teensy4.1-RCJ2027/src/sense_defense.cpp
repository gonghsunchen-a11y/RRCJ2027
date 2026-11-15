#include <Arduino.h>
#include <core_sense.h>

// Defense, sense core
// Line seen (mux 1 or mux 2): PIN_0/PIN_1 HIGH, then lineState (32 bits) to act core
//   bit 0-15  = mux 1 channel 0-15
//   bit 16-31 = mux 2 channel 0-15

#define M2_UNUSED (0x3E00UL << 16) // mux 2 channel 9-13 have no sensor
#define DEBUG 1                    // 1 = print sensor data every 100 ms

// Debug print: raw value and line bit (0 = line) for one mux, channel 0-15
void printMux(const char* name, int first) {
  Serial.print(name);
  for (int ch = 0; ch < 16; ch++) {
    Serial.printf("%5d%c", lsRaw[first + ch], (lineState >> (first + ch)) & 1 ? ' ' : 'L');
  }
  Serial.println();
}

void setup() {
  core_sense_init();
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 255); // line sensor LED brightness (0-255)
}

void loop() {
  fast_update_line_sensor();
  lineState |= M2_UNUSED; // never report a line on unused channels

  if (!calibrate_line_sensor()) { // button pin 36 calibrates mux 1
    send_line_state();
    digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF); // LED 13 on = line seen

    static uint32_t lastPrint = 0;
    if (DEBUG && millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.printf("flag %d  state %08lX\n", lineState != 0xFFFFFFFF, lineState);
      printMux("M1:", 0);
      printMux("M2:", 16);
    }
  }
}
