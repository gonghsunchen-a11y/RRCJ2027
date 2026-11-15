#ifndef CORE_SENSE_H
#define CORE_SENSE_H
#include <Arduino.h>

#define LINE_SCAN_BUTTON 36

// 16 IR ports, port N is index N-1
// odd ports  (1,3,..,15): TSOP4038 -> far
// even ports (2,4,..,16): TSSP     -> near
#define IR_COUNT 16
#define LS_BRIGHTNESS 24

// Line sensor: 16-channel mux, select pins S0-S3, analog outputs M1 / M2
#define LS_S0 16
#define LS_S1 17
#define LS_S2 18
#define LS_S3 19
#define LS_M1 14 // channel 0-15
#define LS_M2 15 // reserved, channel 16-31
#define LS_COUNT 16 // channels in use (M1 only), 32 when M2 is used

// Line flag to act core: HIGH while any sensor sees a line (faster than UART)
#define PIN_0 0
#define PIN_1 1

extern const uint8_t IR_PINS[IR_COUNT];
extern uint16_t irValue[IR_COUNT]; // higher = ball closer

extern uint16_t lsRaw[32];    // last analog reading per line sensor channel
extern uint16_t avg_ls[32];   // threshold per line sensor channel
extern uint32_t lineState;    // bit = 0 when reading below threshold for 2 scans in a row

void core_sense_init();
void update_ball_sensor();
void fast_update_line_sensor();
bool calibrate_line_sensor(); // true while scanning
<<<<<<< HEAD
#endif
=======
void send_line_state();       // set PIN_0/PIN_1, send lineState to act core (Serial8)
#endif
>>>>>>> fe52e0dbcfcd07409aaae8c95cdd78b429337788
