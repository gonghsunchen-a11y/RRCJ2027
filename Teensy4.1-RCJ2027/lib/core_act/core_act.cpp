#include <core_act.h>
#include <DMAChannel.h>
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
    pinMode(START_BUTTON, INPUT_PULLUP);

    // Line flag: stop motors the moment either pin goes HIGH, before UART data arrives
    pinMode(PIN_0, INPUT_PULLDOWN);
    pinMode(PIN_1, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_0), MotorStop, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_1), MotorStop, RISING);

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

// ===== Top camera over DMA (Serial3 = LPUART2) =====
// DMA copies every received byte into a ring buffer, no CPU and no interrupt
#define CAM_BUF 64 // power of 2, buffer must be aligned to its size
static volatile uint8_t camRx[CAM_BUF] __attribute__((aligned(CAM_BUF)));
static DMAChannel camDma;

void camera_dma_begin() {
  Serial3.begin(115200);                                  // pins + baud rate
  NVIC_DISABLE_IRQ(IRQ_LPUART2);                          // Serial3 interrupt off,
  LPUART2_CTRL &= ~(LPUART_CTRL_RIE | LPUART_CTRL_ILIE);  // DMA takes the bytes
  LPUART2_WATER = LPUART_WATER_RXWATER(0);                // DMA request for every byte

  camDma.begin(true);
  camDma.source((volatile const unsigned char&)LPUART2_DATA);
  camDma.destinationCircular(camRx, CAM_BUF);             // wraps around, runs forever
  camDma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART2_RX);
  camDma.enable();
  LPUART2_BAUD |= LPUART_BAUD_RDMAE;                      // UART RX -> DMA requests on
}

// Parse the bytes DMA received since the last call
bool read_camera() {
  static uint8_t buf[9];
  static int n = 0;
  static uint32_t tail = 0; // next byte to parse
  bool got = false;

  uint32_t head = (uint32_t)camDma.TCD->DADDR - (uint32_t)camRx; // next byte DMA writes
  while (tail != head) {
    uint8_t b = camRx[tail];
    tail = (tail + 1) % CAM_BUF;
    if (n == 0 && b != 0xCC) continue;                      // wait for start byte
    if (n == 1 && b != 0xDD) { n = (b == 0xCC); continue; } // resync
    buf[n++] = b;
    if (n < 9) continue;

    n = 0;
    uint8_t checksum = 0;
    for (int i = 1; i <= 6; i++) checksum += buf[i];
    if (buf[8] != 0xEE || checksum != buf[7]) continue;

    topmaixData.valid      = true;
    topmaixData.goal_found = buf[2];
    topmaixData.goal_angle = buf[3] | (buf[4] << 8);
    topmaixData.goal_dist  = buf[5] | (buf[6] << 8);
    got = true;
  }
  return got;
}

// ===== Line (from sense core) =====
bool line_flag() {
  return digitalReadFast(PIN_0) || digitalReadFast(PIN_1);
}

// Packet: AA s0 s1 s2 s3 checksum (sum of s0-s3)
bool read_line_state(uint32_t& state) {
  bool got = false;
  while (Serial8.available() >= 6) {
    if (Serial8.peek() != 0xAA) { Serial8.read(); continue; } // resync
    uint8_t b[6];
    Serial8.readBytes(b, 6);
    if ((uint8_t)(b[1] + b[2] + b[3] + b[4]) != b[5]) continue;
    state = b[1] | (b[2] << 8) | (b[3] << 16) | ((uint32_t)b[4] << 24);
    got = true;
  }
  return got;
}

// Average direction of the sensors that see the line (bit = 0)
// Sensors on opposite sides cancel out -> return fallback (if >= 0), else the lowest sensor
float line_angle(uint32_t state, float fallback) {
  float x = 0, y = 0;
  int first = -1;
  for (int ch = 0; ch < LS_COUNT; ch++) {
    if (state & (1UL << ch)) continue;
    if (first < 0) first = ch;
    x += cosf(ch * 22.5f * DEG_TO_RAD);
    y += sinf(ch * 22.5f * DEG_TO_RAD);
  }
  if (first < 0) return -1;
  if (x * x + y * y < 0.25f) return fallback >= 0 ? fallback : first * 22.5f;

  float a = atan2f(y, x) * RAD_TO_DEG;
  return a < 0 ? a + 360.0f : a;
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
  // Inverse kinematics: wheel speed = projection of (vx, vy) on wheel axis + omega
  SetMotorSpeed(1, -0.643f * vx + 0.766f * vy + omega);
  SetMotorSpeed(2, -0.643f * vx - 0.766f * vy + omega);
  SetMotorSpeed(3,  0.707f * vx - 0.707f * vy + omega);
  SetMotorSpeed(4,  0.707f * vx + 0.707f * vy + omega);
}

// Robot-frame motion; rot_V nudges the target heading, clamped to 45..135
void Vector_Motion(float Vx, float Vy, float rot_V) {
  robot.robot_heading = constrain(robot.robot_heading + rot_V, 45.0f, 135.0f);
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