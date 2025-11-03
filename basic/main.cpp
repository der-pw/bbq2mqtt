/*
 * Maverick ET-732 BBQ Thermometer Decoder (ESP32)
 * -----------------------------------------------
 * - 433.92 MHz OOK (Manchester ~2 kbps)
 * - Empfang über 433 MHz Empfänger an GPIO 23
 * - Ausgabe: Temperaturen Probe 1 / Probe 2 auf Serial (115200 baud)
 * - Doppelte Telegramme (Wiederholungen) werden gefiltert
 */

#include <Arduino.h>

// === Einstellungen ===
#define PIN_RX 23           // 433-MHz-Empfänger DATA-Pin
#define STATE_START_PULSES  0
#define STATE_FIRST_BIT     1
#define STATE_DATA          2

// === Globale Variablen ===
volatile unsigned int start_pulse_counter = 0;
volatile unsigned int detection_state = STATE_START_PULSES;
volatile unsigned long last_interrupt_micros = 0;
volatile unsigned long last_interrupt_millis = 0;
volatile unsigned int nibble = 0;
volatile unsigned int data_array_index = 0;

volatile unsigned int shift_value = 0;
volatile unsigned int short_bit = 0;
volatile unsigned int add_1st_bit = 1;
volatile unsigned int current_byte = 0;
volatile unsigned int current_bit = 1;
volatile unsigned int bit_count = 0;


volatile uint8_t data_array[13];
volatile uint8_t save_array[13];


unsigned int probe1 = 0, probe2 = 0;
unsigned int probe1_array[6], probe2_array[6];

// Duplikat-Filter
uint8_t last_save_array[13];
uint32_t last_change_time = 0;

// === Hilfsfunktionen ===
unsigned int quart(unsigned int param) {
  param &= 0x0F;
  if (param == 0x05) return 0;
  if (param == 0x06) return 1;
  if (param == 0x09) return 2;
  if (param == 0x0A) return 3;
  return 0xFF;
}

// === Datenausgabe ===
void outputData() {
  unsigned int i = 0;
  probe1 = probe2 = 0;

  // Header-Check
  if ((save_array[0] != 0xAA) ||
      (save_array[1] != 0x99) ||
      (save_array[2] != 0x95) ||
      (save_array[3] != 0x59)) {
    return;
  }

  // --- Duplikat-Filter ---
  if (memcmp((const void*)save_array, (const void*)last_save_array,
             sizeof(save_array)) == 0) {
    return;  // identisches Telegramm, ignorieren
  }
  memcpy((void*)last_save_array, (const void*)save_array, sizeof(save_array));
  last_change_time = millis();

  // --- Temperaturberechnung ---
  probe2_array[0] = quart(save_array[8] & 0x0F);
  probe2_array[1] = quart(save_array[8] >> 4);
  probe2_array[2] = quart(save_array[7] & 0x0F);
  probe2_array[3] = quart(save_array[7] >> 4);
  probe2_array[4] = quart(save_array[6] & 0x0F);

  probe1_array[0] = quart(save_array[6] >> 4);
  probe1_array[1] = quart(save_array[5] & 0x0F);
  probe1_array[2] = quart(save_array[5] >> 4);
  probe1_array[3] = quart(save_array[4] & 0x0F);
  probe1_array[4] = quart(save_array[4] >> 4);

  for (i = 0; i <= 4; i++) {
    probe1 += probe1_array[i] * (1 << (2 * i));
    probe2 += probe2_array[i] * (1 << (2 * i));
  }

  probe1 -= 532;
  probe2 -= 532;

  Serial.printf("Probe1: %d °C\tProbe2: %d °C\t@%lu µs\n",
                probe1, probe2, micros());
}

// === Interrupt-Handler ===
void IRAM_ATTR handleInterrupt() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();
  unsigned long time_since_last_ms = nowMillis - last_interrupt_millis;
  unsigned long tsl_micros = nowMicros - last_interrupt_micros;

  last_interrupt_micros = nowMicros;
  last_interrupt_millis = nowMillis;

  unsigned int bit_ok = 0;
  bool pinHigh = digitalRead(PIN_RX);

  // --- Preamble-Erkennung ---
  if (detection_state == STATE_START_PULSES) {
    if (((time_since_last_ms > 3) && (time_since_last_ms < 7)) && pinHigh) {
      start_pulse_counter++;
      if (start_pulse_counter == 8) {
        start_pulse_counter = 0;
        detection_state = STATE_FIRST_BIT;
      }
    } else if (tsl_micros > 400) {
      start_pulse_counter = 0;
    }
  }
  else if (detection_state == STATE_FIRST_BIT && pinHigh) {
    detection_state = STATE_DATA;
    current_bit = 1;
    current_byte = 0;
    shift_value = 0;
    data_array_index = 0;
    short_bit = 0;
    add_1st_bit = 1;
    bit_count = 1;
  }

  // --- Daten-Decodierung ---
  if (detection_state == STATE_DATA) {
    if ((tsl_micros > 90) && (tsl_micros < 390)) {
      if (short_bit == 0) short_bit = 1;
      else { short_bit = 0; bit_ok = 1; }
    }

    if ((tsl_micros > 390) && (tsl_micros < 650)) {
      if (short_bit == 1) {
        detection_state = STATE_START_PULSES;
      }
      bit_count++;
      current_bit = pinHigh;
      bit_ok = 1;
    }

    if (bit_ok) {
      if (add_1st_bit) {
        current_byte = 0x01;
        shift_value = 1;
        add_1st_bit = 0;
      }

      current_byte = (current_byte << 1) + current_bit;
      shift_value++;
      nibble = current_byte;

      if (shift_value == 8) {
        if (data_array_index < 13) data_array[data_array_index++] = current_byte;
        bit_count = 0;
        shift_value = 0;
        current_byte = 0;
      }

      if (data_array_index >= 9) {
        start_pulse_counter = 0;
        detection_state = STATE_START_PULSES;

        for (uint8_t i = 0; i < 9; i++) save_array[i] = data_array[i];
        outputData();
      }
      bit_ok = 0;
    }
  }
}

// === Setup / Loop ===
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nMaverick ET-732 Decoder (ESP32)");
  Serial.println("Empfänger an GPIO 23");

  pinMode(PIN_RX, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RX), handleInterrupt, CHANGE);
}

void loop() {
  delay(100);
}
