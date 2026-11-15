#include <Arduino.h>
#include <core_act.h>

// ===== UI v1 =====
// Screen layout (128x64):
//   y 0-10  : top bar, gyro heading + page number (always shown)
//   y 12-63 : page body, reserved for other data
#define UI_BODY_Y 12
int uiPage = 0;

void drawTopBar() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("H ");
  display.print(gyroData.heading, 1);
  display.setCursor(110, 0);
  display.print(uiPage + 1);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
}

void drawPage() {
  display.clearDisplay();
  drawTopBar();
  switch (uiPage) {
    case 0: break; // body reserved, draw from y = UI_BODY_Y
  }
  display.display();
}

// ===== Main =====
void setup() {
  core_act_init();

  // Restart caused by a software crash? (empty after power loss / brownout)
  if (CrashReport) {
    Serial.print(CrashReport);
    drawMessage("CRASHED");
    delay(2000);
  }

  // Wait for button 1, motors stay stopped; runs until restart after that
  MotorStop();
  drawMessage("Press BTN");
  Serial.println("Press button to start");
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
  }
  delay(200); // debounce
  Serial.println("Start");
}

// ===== Line avoiding test =====
// No line: move forward
// Line seen: motors stop (interrupt), wait for UART data, then move away from the line
#define FORWARD_SPEED 50
#define AVOID_SPEED 40
/*
void loop() {
  static bool avoiding = false;
  static float awayAngle = 0;

  update_gyro_sensor(); // Vector_Motion holds heading with the gyro
  digitalWriteFast(LED_BUILTIN, line_flag()); // LED 13 on = flag from sense core received

  static uint32_t state = 0xFFFFFFFF; // last received line state

  if (line_flag()) {
    // Update direction on every packet; first line direction is used only when
    // sensors on opposite sides cancel out (robot on top of the line)
    static float firstLine = -1;
    if (read_line_state(state)) {
      if (!avoiding) firstLine = line_angle(state);
      float lineDir = line_angle(state, firstLine);
      awayAngle = lineDir + 180.0f;
      if (!avoiding) Serial.printf("Line %04lX at %.1f deg, move to %.1f deg\n", state & 0xFFFF, lineDir, awayAngle);
      avoiding = true;
    }

    if (avoiding) {
      Vector_Motion(AVOID_SPEED * cosf(awayAngle * DEG_TO_RAD),
                    AVOID_SPEED * sinf(awayAngle * DEG_TO_RAD), 0);
    } 
    else {
      //MotorStop(); // waiting for UART data
      Vector_Motion(0, 0, 0); // stop motors while waiting for UART data
    }
  } 
  else {
    if (avoiding) Serial.println("Line cleared");
    avoiding = false;
    Vector_Motion(0, FORWARD_SPEED, 0); // no line: move forward (90 deg)
    while (Serial8.available()) Serial8.read(); // drop old packets
  }

  // Status every 500 ms: flag pins and UART bytes waiting
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus >= 50) {
    lastStatus = millis();
    Serial.printf("PIN0 %d  PIN1 %d  avoiding %d  Serial8 bytes %d  LINE:",
                  digitalReadFast(PIN_0), digitalReadFast(PIN_1), avoiding, Serial8.available());
    for (int ch = 0; ch < LS_COUNT; ch++) Serial.print((state >> ch) & 1); // 0 = sensor sees line
    Serial.println();
  }

  // OLED refresh is slow (~25 ms), so only redraw every 100 ms
  static uint32_t lastDisplay = 0;
  if (millis() - lastDisplay >= 100) {
    lastDisplay = millis();
    drawPage();
  }
}*/

void loop(){
  update_gyro_sensor(); // Vector_Motion holds heading with the gyro
  Vector_Motion(0, 30, 0); // no line: move forward (90 deg)
}

