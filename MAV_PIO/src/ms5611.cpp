#include "ms5611.h"
#include <Arduino.h>
#include <SPI.h>

// Outputs updated by baro task
volatile float ms5611_temp_c = NAN;
volatile float ms5611_pressure_mbar = NAN;
volatile float altitude_asl_m = NAN;
volatile float height_agl_m = NAN;

float altitude_filtered = 0.0f;
const float ALT_ALPHA = 0.12f;

uint16_t C[8] = {0};
float ms5611_temp_offset_c = 0.0f;

// Reference for barometric altitude equation
float p0_ref_mbar = 1013.25f;
float t0_ref_k    = 288.15f;
float ref_alt_asl_m = 0.0f;
bool refSet = false;

static constexpr uint8_t CMD_RESET    = 0x1E;
static constexpr uint8_t CMD_PROM_RD  = 0xA0;
static constexpr uint8_t CMD_ADC_READ = 0x00;

// OSR=1024
static constexpr uint8_t CMD_CONV_D1 = 0x44; // pressure
static constexpr uint8_t CMD_CONV_D2 = 0x54; // temperature
static constexpr uint32_t CONV_TIME_MS = 3;

const int CS_PIN  = 10;
const int SCK_PIN = 12;
const int MOSI_PIN = 11;
const int MISO_PIN = 13;

// Baro state machine
uint8_t baro_state_index = 0;       // 0=temp, 1-4=pressure
uint32_t baro_conv_start_ms = 0;
bool baro_waiting = false;
uint32_t last_D1 = 0;
uint32_t last_D2 = 0;
bool have_D2 = false;

void ms5611_reset() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_RESET);
  digitalWrite(CS_PIN, HIGH);
  delay(4);
}

void ms5611_read_prom() {
  for (int i = 0; i < 8; i++) {
    uint8_t addr = CMD_PROM_RD + (i * 2);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(addr);
    uint8_t msb = SPI.transfer(0x00);
    uint8_t lsb = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    C[i] = (uint16_t(msb) << 8) | lsb;
    delay(2);
  }
}

void ms5611_start_conversion(uint8_t cmd) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(cmd);
  digitalWrite(CS_PIN, HIGH);
  baro_conv_start_ms = millis();
  baro_waiting = true;
}

uint32_t ms5611_read_adc() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_ADC_READ);
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b2 = SPI.transfer(0x00);
  uint8_t b3 = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return (uint32_t(b1) << 16) | (uint32_t(b2) << 8) | b3;
}

// -------------------- MS5611 higher-level --------------------
void ms5611_begin() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(10000000);

  ms5611_reset();
  ms5611_read_prom();

  baro_state_index = 0;
  have_D2 = false;
  ms5611_start_conversion(CMD_CONV_D2);
}

void ms5611_compute(float &temperature_c, float &pressure_mbar, uint32_t D1, uint32_t D2) {
  const float c1 = float(C[1]);
  const float c2 = float(C[2]);
  const float c3 = float(C[3]);
  const float c4 = float(C[4]);
  const float c5 = float(C[5]);
  const float c6 = float(C[6]);

  float dT = float(int32_t(D2) - int32_t(c5 * 256.0f));
  float TEMP = 2000.0f + (dT * c6) / 8388608.0f;                  // 0.01°C
  float OFF  = c2 * 65536.0f + (c4 * dT) / 128.0f;
  float SENS = c1 * 32768.0f + (c3 * dT) / 256.0f;

  if (TEMP < 2000.0f) {
    float T2 = (dT * dT) / 2147483648.0f;
    float aux = (TEMP - 2000.0f) * (TEMP - 2000.0f);
    float OFF2  = 2.5f * aux;
    float SENS2 = 1.25f * aux;

    if (TEMP < -1500.0f) {
      float aux2 = (TEMP + 1500.0f) * (TEMP + 1500.0f);
      OFF2  += 7.0f  * aux2;
      SENS2 += 5.5f  * aux2;
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;
  }

  float P = (float(D1) * SENS / 2097152.0f - OFF) / 32768.0f;     // 0.01 mbar
  temperature_c = (TEMP * 0.01f) + ms5611_temp_offset_c;
  pressure_mbar = P * 0.01f;
}

void ms5611_task_update() {
  if (!baro_waiting) return;
  if (millis() - baro_conv_start_ms < CONV_TIME_MS) return;

  baro_waiting = false;
  uint32_t adc = ms5611_read_adc();
  if (adc == 0 || adc == 0xFFFFFF) {
    if (baro_state_index == 0) ms5611_start_conversion(CMD_CONV_D2);
    else ms5611_start_conversion(CMD_CONV_D1);
    return;
  }

  if (baro_state_index == 0) {
    last_D2 = adc;
    have_D2 = true;
    baro_state_index = 1;
    ms5611_start_conversion(CMD_CONV_D1);
    return;
  }

  last_D1 = adc;

  if (have_D2) {
    float t_c, p_mbar;
    ms5611_compute(t_c, p_mbar, last_D1, last_D2);

    ms5611_temp_c = t_c;
    ms5611_pressure_mbar = p_mbar;

    if (!refSet && isfinite(p_mbar) && isfinite(t_c)) {
      calibrateAltitudeReference(p_mbar, t_c, 0.0f);
    }

    if (refSet && isfinite(p_mbar)) {
      const float dh = calcDeltaH_m(p_mbar);
      height_agl_m = dh;
      altitude_asl_m = ref_alt_asl_m + dh;
      altitude_asl_m = filterAltitude(altitude_asl_m);
    }
  }

  baro_state_index++;
  if (baro_state_index > 4) {
    baro_state_index = 0;
    ms5611_start_conversion(CMD_CONV_D2);
  } else {
    ms5611_start_conversion(CMD_CONV_D1);
  }
}

// -------------------- Altitude math --------------------
void calibrateAltitudeReference(float pressure_mbar, float temp_c, float altitude_asl_m_in) {
  p0_ref_mbar = pressure_mbar;
  t0_ref_k    = temp_c + 273.15f;
  ref_alt_asl_m = altitude_asl_m_in;
  refSet = true;
}

float calcDeltaH_m(float pressure_mbar) {
  if (!refSet || !isfinite(pressure_mbar) || pressure_mbar <= 0) return 0.0f;
  const float EXP = 0.190263f;
  float ratio = p0_ref_mbar / pressure_mbar;
  float dh = (t0_ref_k / 0.0065f) * (powf(ratio, EXP) - 1.0f);
  return dh;
}

float filterAltitude(float a) {
  static bool first = true;
  if (first) { first = false; altitude_filtered = a; }
  altitude_filtered = ALT_ALPHA * a + (1.0f - ALT_ALPHA) * altitude_filtered;
  return altitude_filtered;
}