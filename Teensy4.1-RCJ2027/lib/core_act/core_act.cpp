#include <core_act.h>
GyroData gyroData;
TopMaixData topmaixData;
RobotState robot;
// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const uint8_t motorDirPins[MOTOR_COUNT] = MOTOR_DIR_PINS;
static const uint8_t motorPwmPins[MOTOR_COUNT] = MOTOR_PWM_PINS;

void core_act_init() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial3.setTimeout(50); // ms, camera answers once per frame
    Serial8.begin(921600);
    pinMode(LED_BUILTIN, OUTPUT);

    // Motor Initialization
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motorDirPins[i], OUTPUT);
        pinMode(motorPwmPins[i], OUTPUT);
        analogWrite(motorPwmPins[i], 0);
    }
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH);   // wake motor drivers

    pinMode(BALL_POSESSION, INPUT);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C) &&
        !display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        Serial.println("OLED init failed at I2C addresses 0x3C and 0x3D");
        for (;;) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(250);
        }
    }
    Serial.println("OLED initialized");

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawMessage("Starting");
}

void update_gyro_sensor(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.exist = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 (uint8_t wraps) ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    if(esti_checksum != buffer[18]) continue;

    // --- Extract angles (Little Endian, 0.01 deg) ---
    int16_t yaw_raw   = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);
    int16_t roll_raw  = (int16_t)((buffer[8] << 8) | buffer[7]);

    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.exist = true;
    }
    if(abs(pitch_raw) <= 18000) gyroData.pitch = pitch_raw * 0.01f;
    if(abs(roll_raw)  <= 18000) gyroData.roll  = roll_raw  * 0.01f;
    // Keep reading: drain all queued packets so the newest one wins
  }
}

// Packet: CC DD found angleL angleH distL distH checksum EE, checksum = sum of bytes [1..6]
void readTopMaix_polar() {
  uint8_t buf[9];
  while (Serial3.available()) Serial3.read(); // drop stale bytes
  Serial3.write(0xDD);                        // request

  uint8_t checksum = 0;
  bool ok = Serial3.readBytes(buf, 9) == 9 &&
            buf[0] == 0xCC && buf[1] == 0xDD && buf[8] == 0xEE;
  for (int i = 1; i <= 6; i++) checksum += buf[i];

  topmaixData.valid = ok && checksum == buf[7];
  if (!topmaixData.valid) return;

  topmaixData.goal_found = buf[2];
  topmaixData.goal_angle = buf[3] | (buf[4] << 8);
  topmaixData.goal_dist  = buf[5] | (buf[6] << 8);
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}

// ===== Motors =====
void SetMotorSpeed(uint8_t port, float speed){
  if (port < 1 || port > MOTOR_COUNT) return;
  uint8_t i = port - 1;

  speed = constrain(speed, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
  if (speed == 0) {
    analogWrite(motorPwmPins[i], 0);
    return;
  }
  //digitalWrite(motorDirPins[i], speed > 0 ? LOW : HIGH);
  digitalWrite(motorDirPins[i], speed < 0 ? LOW : HIGH);
  analogWrite(motorPwmPins[i], (int)(fabsf(speed) * 255.0f / 100.0f));
}

void MotorStop(){
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    analogWrite(motorPwmPins[i], 0);
  }
}

// ===== Motion =====
// Wrap angle to (-180, 180]
static float wrap180(float a) {
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// Current heading in the 90-forward convention
static float currentHeading() {
  return 90.0f - gyroData.heading;
}

// P controller with deadband: rotation command to reach target heading
static float headingCorrection(float target_heading) {
  float e = wrap180(target_heading - currentHeading());
  return (fabsf(e) > robot.heading_threshold) ? e * robot.P_factor : 0.0f;
}

void RobotIKControl(float vx, float vy, float omega) {
  // Invert translation signs for the positive-right/positive-forward command convention.
  Serial.printf("vx = %f, vy = %f\n", vx, vy);
  SetMotorSpeed(1,  -0.643f * vx + 0.766f * vy + omega);
  SetMotorSpeed(2,  -0.643f * vx - 0.766f * vy + omega);
  SetMotorSpeed(3, 0.707f * vx - 0.707f * vy + omega);
  SetMotorSpeed(4, 0.707f * vx + 0.707f * vy + omega);
}

// Robot-frame motion; rot_V nudges the target heading, clamped to 45..135
void Vector_Motion(float Vx, float Vy, float rot_V) {
  Serial.printf("vector motion %f, %f\n", Vx, Vy);
  robot.robot_heading = constrain(robot.robot_heading + rot_V, 70.0f, 110.0f);
  RobotIKControl(Vx, Vy, headingCorrection(robot.robot_heading));
}

// Field-centric motion: (WVx, WVy) in field frame, robot turns to target_heading
void FC_Vector_Motion(float WVx, float WVy, float target_heading) {
  // Rotate by the *current* heading so translation stays field-aligned while turning
  float rad = (currentHeading() - 90.0f) * DEG_TO_RAD;
  float cos_h = cosf(rad);
  float sin_h = sinf(rad);

  float robot_vx =  WVx * cos_h + WVy * sin_h;
  float robot_vy = -WVx * sin_h + WVy * cos_h;

  RobotIKControl(robot_vx, robot_vy, headingCorrection(target_heading));
}