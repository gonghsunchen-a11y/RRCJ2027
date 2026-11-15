
#include <Arduino.h>
#include <DMAChannel.h>
/*
// New camera protocol (Serial3, 115200): camera sends a packet every frame, no request
//   CC DD found angleL angleH distL distH checksum EE   (checksum = sum of bytes 1-6)
// Serial3 RX uses DMA: every received byte is copied by DMA (no CPU, no interrupt)
// into a circular buffer. readCamera() never waits, it only parses new bytes.
// Prints every packet (time since the previous one) and a summary every 100 packets.

// ===== Serial3 (LPUART2) RX DMA =====
#define CAM_BUF 64 // power of 2, buffer must be aligned to its size
volatile uint8_t camRx[CAM_BUF] __attribute__((aligned(CAM_BUF)));
DMAChannel camDma;

void cameraDmaBegin() {
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

struct CamData {
  bool found;
  uint16_t angle; // deg 0-359, 0xFFFF = not found
  uint16_t dist;  // 0xFFFF = not found
};
CamData cam;
uint32_t camBad = 0; // packets with wrong end byte or checksum

// Non-blocking. Returns true when a new valid packet was parsed into cam
bool readCamera() {
  static uint8_t buf[9];
  static int n = 0;
  static uint32_t tail = 0; // next byte to parse
  bool got = false;

  uint32_t head = (uint32_t)camDma.TCD->DADDR - (uint32_t)camRx; // next byte DMA writes
  while (tail != head) {
    uint8_t b = camRx[tail];
    tail = (tail + 1) % CAM_BUF;
    if (n == 0 && b != 0xCC) continue;                 // wait for start byte
    if (n == 1 && b != 0xDD) { n = (b == 0xCC); continue; } // resync
    buf[n++] = b;
    if (n < 9) continue;

    n = 0;
    uint8_t checksum = 0;
    for (int i = 1; i <= 6; i++) checksum += buf[i];
    if (buf[8] != 0xEE || checksum != buf[7]) { camBad++; continue; }

    cam.found = buf[2];
    cam.angle = buf[3] | (buf[4] << 8);
    cam.dist  = buf[5] | (buf[6] << 8);
    got = true;
  }
  return got;
}

void setup() {
  Serial.begin(115200);
  cameraDmaBegin();
}

void loop() {
  static uint32_t lastPacket = micros();
  static uint32_t iMin = 0xFFFFFFFF, iMax = 0, iSum = 0, count = 0, readMax = 0;

  uint32_t t0 = micros();
  bool got = readCamera();
  readMax = max(readMax, micros() - t0); // cost of one readCamera() call
  if (!got) return;

  uint32_t now = micros();
  uint32_t interval = now - lastPacket;
  lastPacket = now;

  iSum += interval;
  iMin = min(iMin, interval);
  iMax = max(iMax, interval);
  Serial.printf("%6lu us | found %d angle %d dist %d\n", interval, cam.found, cam.angle, cam.dist);

  if (++count == 100) {
    Serial.printf("== 100 packets: interval min %lu  avg %lu  max %lu us | bad %lu | readCamera max %lu us ==\n",
                  iMin, iSum / count, iMax, camBad, readMax);
    iMin = 0xFFFFFFFF; iMax = 0; iSum = 0; count = 0; camBad = 0; readMax = 0;
  }
}
*/
// ===== Old test, request protocol (results below) =====
// Timing test, current camera protocol (Serial3, 115200):
//   send 0xDD -> camera replies CC DD found angleL angleH distL distH checksum EE
// Measures the time from request to a complete, valid packet.
// Prints every packet, and min / avg / max every 100 requests.

#define TIMEOUT_US 100000 // 100 ms, give up on a request

// Send a request and wait for one packet. Returns time in us, or -1 on timeout / bad packet
long requestPacket(uint8_t* buf) {
  while (Serial3.available()) Serial3.read(); // drop stale bytes

  uint32_t start = micros();
  Serial3.write(0xDD);

  int n = 0;
  while (n < 9) {
    if (micros() - start > TIMEOUT_US) return -1;
    if (Serial3.available()) buf[n++] = Serial3.read();
  }
  uint32_t t = micros() - start;

  uint8_t checksum = 0;
  for (int i = 1; i <= 6; i++) checksum += buf[i];
  bool ok = buf[0] == 0xCC && buf[1] == 0xDD && buf[8] == 0xEE && checksum == buf[7];
  return ok ? (long)t : -1;
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  static uint32_t tMin = 0xFFFFFFFF, tMax = 0, tSum = 0, good = 0, bad = 0;
  uint8_t buf[9];

  long t = requestPacket(buf);
  if (t < 0) {
    bad++;
    Serial.println("timeout / bad packet");
  } else {
    good++;
    tSum += t;
    tMin = min(tMin, (uint32_t)t);
    tMax = max(tMax, (uint32_t)t);
    Serial.printf("%6ld us | found %d angle %d dist %d\n",
                  t, buf[2], buf[3] | (buf[4] << 8), buf[5] | (buf[6] << 8));
  }

  if (good + bad == 100) {
    Serial.printf("== 100 requests: ok %lu, fail %lu | min %lu  avg %lu  max %lu us ==\n",
                  good, bad, good ? tMin : 0, good ? tSum / good : 0, tMax);
    tMin = 0xFFFFFFFF; tMax = 0; tSum = 0; good = 0; bad = 0;
  }
}