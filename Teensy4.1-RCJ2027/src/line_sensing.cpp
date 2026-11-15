#include <Arduino.h>
#include <core_sense.h>

// Read line sensor channel 0-15 and print the raw values
void setup() {
    Serial.begin(115200);
    pinMode(LS_S0, OUTPUT);
    pinMode(LS_S1, OUTPUT);
    pinMode(LS_S2, OUTPUT);
    pinMode(LS_S3, OUTPUT);
    pinMode(LS_BRIGHTNESS, OUTPUT);
    //pinMode(LS_M1, INPUT_PULLUP);
    pinMode(LS_BRIGHTNESS, OUTPUT);
    //analogWriteFrequency(LS_BRIGHTNESS, 200000); // 200 kHz, much faster than one ADC read
    analogWriteFrequency(LS_BRIGHTNESS, 20000);  // was 200000

    analogWrite(LS_BRIGHTNESS, 155); // line sensor LED brightness (0-255)
}

// Print channel 0-15 on one line
void loop() {
  
  for (int ch = 0; ch < 16; ch++) {
    digitalWrite(LS_S0, (ch >> 0) & 1);
    digitalWrite(LS_S1, (ch >> 1) & 1);
    digitalWrite(LS_S2, (ch >> 2) & 1);
    digitalWrite(LS_S3, (ch >> 3) & 1);
    delayMicroseconds(10); // mux settle

    Serial.printf("%4d", analogRead(LS_M1));
    Serial.print(' ');
  }
  Serial.printf("%4d", analogRead(LS_M1));
  Serial.println();
  delay(50);
}
