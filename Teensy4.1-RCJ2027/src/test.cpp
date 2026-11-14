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


void setup(){
    core_act_init();
}
void loop(){
  update_gyro_sensor();
  Serial.println(gyroData.heading);

  readTopMaix_polar();
  Vector_Motion(0,0,0);
  //SetMotorSpeed(4, 20);

}
