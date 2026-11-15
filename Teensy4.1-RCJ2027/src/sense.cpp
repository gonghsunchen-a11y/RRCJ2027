#include <Arduino.h>
#include <core_sense.h>

// Print one sensor group and its strongest port
// firstPort = 1 -> far  (odd ports 1,3,..,15)
// firstPort = 2 -> near (even ports 2,4,..,16)
void printIrGroup(const char* name, int firstPort) {
  int maxPort = 0;
  int maxValue = 0;

  Serial.print(name);
  for (int port = firstPort; port <= IR_COUNT; port += 2) {
    int value = irValue[port - 1];
    Serial.printf(" P%d:%d", port, value);
    if (value > maxValue) {
      maxValue = value;
      maxPort = port;
    }
  }

  if (maxPort == 0) {
    Serial.println("  | best: none");
  } else {
    Serial.printf("  | best: P%d (pin %d) = %d\n", maxPort, IR_PINS[maxPort - 1], maxValue);
  }
}

void setup() {
  core_sense_init();
  analogWrite(LS_BRIGHTNESS, 155); // line sensor LED brightness (0-255)
}

void loop() {
  bool scanning = false;

  // ===== Line sensor (disabled) =====
  fast_update_line_sensor();
  scanning = calibrate_line_sensor(); // prints its own data while scanning
  if (!scanning) {
    send_line_state();                // flag pins + UART to act core
    digitalWriteFast(LED_BUILTIN, (lineState & 0xFFFF) != 0xFFFF); // LED 13 on = line seen
  }


  // ===== IR ball sensor =====
  update_ball_sensor();

  // Print every 100 ms, the sensors keep running at full speed
  static uint32_t lastPrint = 0;
  if (!scanning && millis() - lastPrint >= 100) {
    lastPrint = millis();
    printIrGroup("FAR :", 1); // TSSP4038
    printIrGroup("NEAR:", 2); // TSOP4840

    Serial.print("LINE:");
    for (int ch = 0; ch < 16; ch++) {
      Serial.print((lineState >> ch) & 1);
    }
    Serial.println();
    

    Serial.println();
  }
}