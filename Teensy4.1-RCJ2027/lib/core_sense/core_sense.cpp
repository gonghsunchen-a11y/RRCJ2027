#include <core_sense.h>
#include <EEPROM.h>

const uint8_t IR_PINS[IR_COUNT] = {
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, // port 1  - 11
    23, 22, 21,                          // port 12 - 14
    28, 29                               // port 15 - 16
};
uint16_t irValue[IR_COUNT];

uint16_t lsRaw[32];
uint16_t avg_ls[32];
uint32_t lineState = 0xFFFFFFFF;

void core_sense_init() {
    Serial.begin(115200);
    Serial8.begin(921600); // to act core
    pinMode(PIN_0, OUTPUT);
    pinMode(PIN_1, OUTPUT);
    digitalWrite(PIN_0, LOW);
    digitalWrite(PIN_1, LOW);
    pinMode(LINE_SCAN_BUTTON, INPUT_PULLUP);
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
    pinMode(LS_BRIGHTNESS, OUTPUT);
    analogWriteFrequency(LS_BRIGHTNESS, 200000); // 200 kHz, much faster than one ADC read

    pinMode(LS_S0, OUTPUT);
    pinMode(LS_S1, OUTPUT);
    pinMode(LS_S2, OUTPUT);
    pinMode(LS_S3, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LS_M1, INPUT);
    pinMode(LS_M2, INPUT);

    // Load thresholds saved by calibrate_line_sensor(); erased EEPROM reads 0xFFFF
    // M1 = avg_ls[0-15], M2 = avg_ls[16-31] (saved by mux2_test)
    EEPROM.get(0, avg_ls);
    for (int i = 0; i < 32; i++) {
        if (avg_ls[i] == 0xFFFF) avg_ls[i] = 512;
    }
}

// Count LOW samples on each sensor over 1 ms (one ball pulse cycle is ~833 us)
void update_ball_sensor() {
    for (int i = 0; i < IR_COUNT; i++) irValue[i] = 0;

    uint32_t start = micros();
    while (micros() - start < 1000) {
        for (int i = 0; i < IR_COUNT; i++) {
            if (digitalRead(IR_PINS[i]) == LOW) irValue[i]++;
        }
    }
}

void fast_update_line_sensor() {
    static uint32_t prevRaw = 0xFFFFFFFF;
    uint32_t rawState       = 0xFFFFFFFF;

    for (uint8_t ch = 0; ch < 16; ch++) {
        digitalWriteFast(LS_S0, (ch >> 0) & 1);
        digitalWriteFast(LS_S1, (ch >> 1) & 1);
        digitalWriteFast(LS_S2, (ch >> 2) & 1);
        digitalWriteFast(LS_S3, (ch >> 3) & 1);
        /*might be allowed to removed*/
        delayMicroseconds(10);

        uint16_t r1 = analogRead(LS_M1);
        uint16_t r2 = analogRead(LS_M2);
        lsRaw[ch] = r1;
        lsRaw[ch + 16] = r2;

        if (r1 < avg_ls[ch])    rawState &= ~(1UL << ch);
        if (r2 < avg_ls[ch+16]) rawState &= ~(1UL << (ch + 16));
    }

    // Bit clears only if below threshold in this scan AND the previous one
    lineState = prevRaw | rawState;
    prevRaw   = rawState;
}

// Call every loop after fast_update_line_sensor().
// Line seen: PIN_0/PIN_1 HIGH, then send packet AA s0 s1 s2 s3 checksum (sum of s0-s3)
// No line:   PIN_0/PIN_1 LOW, nothing sent
void send_line_state() {
    bool line = lineState != 0xFFFFFFFF; // M1 (bit 0-15) or M2 (bit 16-31)
    digitalWriteFast(PIN_0, line);
    digitalWriteFast(PIN_1, line);
    if (!line) return;

    uint8_t pkt[6] = {0xAA,
                      (uint8_t)(lineState),       (uint8_t)(lineState >> 8),
                      (uint8_t)(lineState >> 16), (uint8_t)(lineState >> 24), 0};
    for (int i = 1; i <= 4; i++) pkt[5] += pkt[i];
    Serial8.write(pkt, 6);
}

// Call every loop after fast_update_line_sensor().
// 1st press: LED13 blinks, record min/max of each sensor
// 2nd press: LED13 off, save avg_ls to EEPROM, show result for 2 s
// Returns true while scanning
bool calibrate_line_sensor() {
    static bool scanning = false;
    static bool lastBtn = HIGH;
    static uint32_t lastPress = 0;
    static uint32_t blinkTimer = 0;
    static uint16_t max_ls[32], min_ls[32];

    bool btn = digitalRead(LINE_SCAN_BUTTON);
    if (lastBtn == HIGH && btn == LOW && millis() - lastPress > 200) { // pressed, 200 ms debounce
        lastPress = millis();
        scanning = !scanning;

        if (scanning) {
            for (int i = 0; i < 32; i++) {
                max_ls[i] = 0;
                min_ls[i] = 0xFFFF;
            }
            Serial.println("Line scan start");
        } else {
            digitalWrite(LED_BUILTIN, LOW);
            for (int i = 0; i < LS_COUNT; i++) {
                avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
                Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
            }
            EEPROM.put(0, avg_ls);
            Serial.println("Line scan saved");
            delay(2000); // keep result on screen
        }
    }
    lastBtn = btn;

    if (scanning) {
        for (int i = 0; i < LS_COUNT; i++) {
            if (lsRaw[i] > max_ls[i]){
                max_ls[i] = lsRaw[i];
                Serial.printf("%d find max %d\n", i, max_ls[i]);
            }
            if (lsRaw[i] < min_ls[i]){
                min_ls[i] = lsRaw[i];
                Serial.printf("%d find min %d\n", i, min_ls[i]);
            } 
        }

        // Blink LED and print raw data every 100 ms
        if (millis() - blinkTimer > 50) {
            blinkTimer = millis();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            Serial.print("RAW:");
            for (int i = 0; i < LS_COUNT; i++) {
                Serial.print(' ');
                Serial.print(lsRaw[i]);
            }
            Serial.println();
        }
    }
    return scanning;
}