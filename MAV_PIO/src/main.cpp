/*
   MARS BOARD MAV - ESP32-S3 Flight Computer (PlatformIO)

   SAFE MODE (No USB MSC):
   - Pre-launch RAM cache (ring buffer) so you get data before launch
   - Launch detection -> open a new CSV and dump cache
   - Apogee detection -> mark event in CSV
   - Landing detection -> log a bit more -> close file
   - After file is closed -> start WiFi AP + Web dashboard + download endpoints

   MS5611:
   - Non-blocking sampling state machine (no delays in handlers)
   - Second-order temperature compensation
   - ASL altitude: altitude_asl = ref_alt_asl + delta_h
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FFat.h>
#include <esp_system.h> // esp_random()

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

// -------------------- PIN CONFIG --------------------
const int SDA_PIN = 8;
const int SCL_PIN = 9;

const int CS_PIN  = 10;
const int SCK_PIN = 12;
const int MOSI_PIN = 11;
const int MISO_PIN = 13;

const int LED_PIN = 39;

// -------------------- ACCESS POINT --------------------
const char* ap_ssid = "MAV_TELEM";
const char* ap_pass = "RocketGroup1!"; // must be >=8 chars
WebServer server(80);
bool apMode = false;

// If true: AP runs pre-launch for setup; turns off during flight; turns back on after landing.
static constexpr bool WIFI_ON_GROUND = true;
static constexpr bool WIFI_OFF_DURING_FLIGHT = false;

// -------------------- MPU6050 DMP --------------------
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
uint16_t packetSize = 0;

Quaternion q;
VectorFloat gravity;
float ypr[3];   // yaw, pitch, roll (rad)
float ypr_offset[3] = {0, 0, 0};

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t temp_raw;
float temp_mpu;

// MPU6050 scaling
const float ACCEL_SCALE = 2048.0f; // ±16g
const float GYRO_SCALE  = 16.4f;   // ±2000deg/s

// Device offsets
const int16_t DEV_ACCEL_X_OFFSET = 844;
const int16_t DEV_ACCEL_Y_OFFSET = 1341;
const int16_t DEV_ACCEL_Z_OFFSET = 743;
const int16_t DEV_GYRO_X_OFFSET  = 30;
const int16_t DEV_GYRO_Y_OFFSET  = 67;
const int16_t DEV_GYRO_Z_OFFSET  = -3;

// -------------------- MS5611 / ALTITUDE --------------------
static constexpr uint8_t CMD_RESET    = 0x1E;
static constexpr uint8_t CMD_PROM_RD  = 0xA0;
static constexpr uint8_t CMD_ADC_READ = 0x00;

// OSR=1024
static constexpr uint8_t CMD_CONV_D1 = 0x44; // pressure
static constexpr uint8_t CMD_CONV_D2 = 0x54; // temperature
static constexpr uint32_t CONV_TIME_MS = 3;

uint16_t C[8] = {0};
float ms5611_temp_offset_c = 0.0f;

// Reference for barometric altitude equation
float p0_ref_mbar = 1013.25f;
float t0_ref_k    = 288.15f;
float ref_alt_asl_m = 0.0f;
bool refSet = false;

// Outputs updated by baro task
volatile float ms5611_temp_c = NAN;
volatile float ms5611_pressure_mbar = NAN;
volatile float altitude_asl_m = NAN;
volatile float height_agl_m = NAN;

// Simple altitude low-pass
float altitude_filtered = 0.0f;
const float ALT_ALPHA = 0.12f;

// Baro state machine
uint8_t baro_state_index = 0;       // 0=temp, 1-4=pressure
uint32_t baro_conv_start_ms = 0;
bool baro_waiting = false;
uint32_t last_D1 = 0;
uint32_t last_D2 = 0;
bool have_D2 = false;

// -------------------- LED CONTROL --------------------
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_FAST = 150;
const unsigned long BLINK_SLOW = 1000;

// -------------------- FLIGHT STATE / LOGGING --------------------
enum FlightState : uint8_t {
  ST_GROUND = 0,  // cache; (optional) AP on for setup
  ST_FLIGHT = 1,  // logging; AP off
  ST_LANDED = 2,  // post-land logging window
  ST_DONE   = 3   // file closed; AP on for download
};

FlightState flightState = ST_GROUND;

// Logging cadence
static constexpr uint32_t LOG_HZ = 50;
static constexpr uint32_t LOG_INTERVAL_MS = 1000 / LOG_HZ;

// Pre-launch cache
static constexpr uint32_t CACHE_SECONDS = 4;
static constexpr uint16_t CACHE_SAMPLES = LOG_HZ * CACHE_SECONDS;

// Detection tuning (adjust after ground tests)
static constexpr float LAUNCH_ALT_THRESH_M = 8.0f;
static constexpr float LAUNCH_VEL_THRESH_MPS = 8.0f;
static constexpr uint16_t LAUNCH_CONFIRM_SAMPLES = 5;

static constexpr float APOGEE_VEL_NEG_THRESH_MPS = -1.5f;
static constexpr float APOGEE_DROP_THRESH_M = 2.0f;
static constexpr uint16_t APOGEE_CONFIRM_SAMPLES = 8;

static constexpr float LAND_VEL_ABS_THRESH_MPS = 1.5f;
static constexpr float LAND_ALT_RANGE_M = 2.0f;
static constexpr uint32_t LAND_STABLE_MS = 500;
static constexpr uint32_t LAND_AFTER_APOGEE_DELAY_MS = 10000; // 5–10s (pick 5000..10000)

static constexpr uint32_t POST_LAND_LOG_MS = 3000;

struct Sample {
  uint32_t t_ms;
  float h_m;        // height (relative to start), m
  float alt_asl_m;  // ASL altitude, m
  float vel_mps;    // filtered vertical speed, m/s
  float p_mbar;
  float t_ms5611_c;
  float t_mpu_c;
  float ax_g, ay_g, az_g;
  float pitch_deg, roll_deg;
};

Sample cacheBuf[CACHE_SAMPLES];
uint16_t cacheHead = 0;
bool cacheFilled = false;

File logFile;
bool logOpen = false;
char currentLogPath[40] = {0};

uint32_t nextLogMs = 0;

// Detection internals
bool launchDetected = false;
bool apogeeDetected = false;
bool landingDetected = false;

float maxHeight_m = -1e9f;
uint16_t launchConfirm = 0;
uint16_t apogeeConfirm = 0;
uint32_t apogeeAtMs = 0;

uint32_t landStableStartMs = 0;
float landAltRef = 0.0f;
uint32_t landedAtMs = 0;

// Velocity estimation (windowed average altitude -> velocity)
// Computes v = (avg(h) - avg(prev h)) / dt across a window to suppress baro noise.
static constexpr uint8_t VELWIN = 25;   // window length in samples (LOGHZ=50 => 10 samples = 0.20 s)

float hWin[VELWIN] = {0};
uint32_t tWin[VELWIN] = {0};
uint8_t velIdx = 0;
uint8_t velCount = 0;

float lastAvgH = 0.0f;
uint32_t lastAvgT = 0;
float velfilt = 0.0f; // optional extra low-pass after windowing
// Latest computed vertical speed for dashboard (updated in loop logging cadence)
volatile float latest_vel_mps = 0.0f;

// -------------------- FORWARD DECLARATIONS --------------------
void ms5611_reset();
void ms5611_read_prom();
void ms5611_start_conversion(uint8_t cmd);
uint32_t ms5611_read_adc();

void ms5611_begin();
void ms5611_task_update();
void ms5611_compute(float &temperature_c, float &pressure_mbar, uint32_t D1, uint32_t D2);

void calibrateAltitudeReference(float pressure_mbar, float temp_c, float altitude_asl_m_in);
float calcDeltaH_m(float pressure_mbar);
float filterAltitude(float a);

void read_mpu_raw();
void convert_accel_to_g(float &ax_g, float &ay_g, float &az_g);
void convert_gyro_to_degs(float &gx_degs, float &gy_degs, float &gz_degs);
void read_dmp();
void zeroYPR();
void get_adjusted_ypr(float &yaw_adj, float &pitch_adj, float &roll_adj);

void updateLED();
String htmlPage();
void handleRoot();
void handleSetAltitude();
void handleZeroYPR();
void handleData();
void handleDeleteLogs();

void startAPAndServer();
void handleReset();
void stopAPAndServer();

void cachePush(const Sample &s);
void dumpCacheToFile();
bool openNewLogFile();
void closeLogFileSafe();
void writeCsvHeader();
void writeCsvRow(const Sample &s, const char* eventTag);

float computeVerticalSpeed(float h_now, uint32_t now_ms);
Sample makeSample(float vel_mps_now);
const char* flightStateName(FlightState st);

void updateFlightStateAndLogging(const Sample &s);

/**
 * @brief Reset the MS5611 barometric sensor and wait for it to complete its internal reset.
 *
 * Issues the sensor reset command over SPI and delays briefly to allow the MS5611 to become ready.
 */
void ms5611_reset() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_RESET);
  digitalWrite(CS_PIN, HIGH);
  delay(4);
}

/**
 * @brief Reads the MS5611 PROM calibration coefficients into the global C[] array.
 *
 * Issues PROM read commands over SPI and stores the eight 16-bit calibration words returned by the sensor into C[0..7].
 */
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

/**
 * @brief Initiates an MS5611 ADC conversion using the given command.
 *
 * Sends the specified conversion command to the MS5611 over SPI, records the
 * conversion start time, and marks the barometer as waiting for a result.
 *
 * @param cmd MS5611 conversion command byte (e.g., D1 for pressure or D2 for temperature).
 */
void ms5611_start_conversion(uint8_t cmd) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(cmd);
  digitalWrite(CS_PIN, HIGH);
  baro_conv_start_ms = millis();
  baro_waiting = true;
}

/**
 * @brief Reads the 24-bit ADC conversion result from the MS5611 barometer over SPI.
 *
 * Initiates an ADC read command to the MS5611, retrieves three bytes, and combines them into a 24-bit unsigned value.
 *
 * @return uint32_t 24-bit ADC result (MSB in the high byte of the returned value).
 */
uint32_t ms5611_read_adc() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(CMD_ADC_READ);
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b2 = SPI.transfer(0x00);
  uint8_t b3 = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return (uint32_t(b1) << 16) | (uint32_t(b2) << 8) | b3;
}

/**
 * @brief Initialize the MS5611 barometer and start the first temperature conversion.
 *
 * Configures the SPI interface and chip-select pin for the MS5611, issues a sensor
 * reset, reads the sensor calibration PROM, initializes the internal barometer
 * state variables, and begins a D2 (temperature) conversion cycle.
 */
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

/**
 * @brief Compute compensated temperature and pressure from MS5611 raw ADC readings.
 *
 * Uses stored PROM calibration coefficients and second-order temperature compensation,
 * then applies the configured temperature offset before returning results.
 *
 * @param[out] temperature_c Compensated temperature in degrees Celsius (includes ms5611_temp_offset_c).
 * @param[out] pressure_mbar Compensated pressure in millibar.
 * @param D1 Raw 24-bit pressure ADC reading from the MS5611.
 * @param D2 Raw 24-bit temperature ADC reading from the MS5611.
 */
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

/**
 * @brief Advance the MS5611 non-blocking conversion state machine and update barometer-derived values.
 *
 * Checks whether a pending conversion has completed; if so, reads the ADC result, computes temperature
 * and pressure when both temperature (D2) and pressure (D1) samples are available, and updates the
 * module's public state (ms5611_temp_c, ms5611_pressure_mbar). If no altitude reference is set and
 * valid measurements are available, sets the altitude reference. When a reference exists and pressure
 * is valid, computes AGL and ASL altitude and applies the altitude filter. The function also advances
 * the internal barometer state index and starts the next D1/D2 conversion as required.
 */
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

/**
 * @brief Set the barometric reference values used to compute altitude above sea level.
 *
 * Stores the reference pressure (mbar), reference temperature (converted from °C to K),
 * and the reference altitude ASL (meters), and marks the altitude reference as initialized.
 *
 * @param pressure_mbar Reference absolute pressure in millibars.
 * @param temp_c Reference temperature in degrees Celsius; converted to Kelvin internally.
 * @param altitude_asl_m_in Reference altitude above sea level in meters.
 */
void calibrateAltitudeReference(float pressure_mbar, float temp_c, float altitude_asl_m_in) {
  p0_ref_mbar = pressure_mbar;
  t0_ref_k    = temp_c + 273.15f;
  ref_alt_asl_m = altitude_asl_m_in;
  refSet = true;
}

/**
 * @brief Compute height difference (meters) from the stored pressure/temperature reference using the barometric formula.
 *
 * @param pressure_mbar Current absolute pressure in millibars.
 * @return float Height difference in meters relative to the stored reference altitude (positive when current pressure is lower than the reference). Returns 0.0 if the reference is not set, the input is not finite, or pressure_mbar is non‑positive.
 */
float calcDeltaH_m(float pressure_mbar) {
  if (!refSet || !isfinite(pressure_mbar) || pressure_mbar <= 0) return 0.0f;
  const float EXP = 0.190263f;
  float ratio = p0_ref_mbar / pressure_mbar;
  float dh = (t0_ref_k / 0.0065f) * (powf(ratio, EXP) - 1.0f);
  return dh;
}

/**
 * @brief Smooths altitude readings using a single-pole low-pass filter.
 *
 * The first call initializes the filter state to the provided sample.
 *
 * @param a Current altitude measurement in meters.
 * @return float Filtered altitude in meters.
 */
float filterAltitude(float a) {
  static bool first = true;
  if (first) { first = false; altitude_filtered = a; }
  altitude_filtered = ALT_ALPHA * a + (1.0f - ALT_ALPHA) * altitude_filtered;
  return altitude_filtered;
}

/**
 * @brief Read raw accelerometer, gyroscope, and temperature data from the MPU6050 and update internal state.
 *
 * Reads raw sensor registers into the module's accel (`ax_raw`, `ay_raw`, `az_raw`) and gyro
 * (`gx_raw`, `gy_raw`, `gz_raw`) variables, stores the raw temperature value in `temp_raw`,
 * and updates `temp_mpu` with the sensor temperature in degrees Celsius.
 */
void read_mpu_raw() {
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
  temp_raw = mpu.getTemperature();
  temp_mpu = temp_raw / 340.0f + 36.53f;
}

/**
 * @brief Converts raw accelerometer readings to units of gravity (g).
 *
 * Converts the internally-read raw accelerometer counts into floating-point
 * values expressed in g (multiples of standard gravity) using the configured
 * accelerometer scale factor.
 *
 * @param[out] ax_g X-axis acceleration in g.
 * @param[out] ay_g Y-axis acceleration in g.
 * @param[out] az_g Z-axis acceleration in g.
 */
void convert_accel_to_g(float &ax_g, float &ay_g, float &az_g) {
  ax_g = (float)ax_raw / ACCEL_SCALE;
  ay_g = (float)ay_raw / ACCEL_SCALE;
  az_g = (float)az_raw / ACCEL_SCALE;
}

/**
 * @brief Convert raw gyro samples to angular rates in degrees per second.
 *
 * Converts the last-read raw gyroscope values into degrees per second using
 * the configured GYRO_SCALE and stores the results in the provided references.
 *
 * @param gx_degs Reference to receive the X-axis angular rate in degrees/second.
 * @param gy_degs Reference to receive the Y-axis angular rate in degrees/second.
 * @param gz_degs Reference to receive the Z-axis angular rate in degrees/second.
 */
void convert_gyro_to_degs(float &gx_degs, float &gy_degs, float &gz_degs) {
  gx_degs = (float)gx_raw / GYRO_SCALE;
  gy_degs = (float)gy_raw / GYRO_SCALE;
  gz_degs = (float)gz_raw / GYRO_SCALE;
}

/**
 * @brief Processes available MPU6050 DMP FIFO packets to update orientation and raw sensor readings.
 *
 * If the DMP is not ready this function does nothing. On FIFO overflow it resets the FIFO and aborts.
 * Otherwise it consumes each complete DMP packet, updates the global quaternion, gravity vector, and
 * yaw/pitch/roll values, and calls read_mpu_raw() to refresh raw accelerometer/gyroscope/temperature samples.
 */
void read_dmp() {
  if (!dmpReady) return;

  uint8_t intStatus = mpu.getIntStatus();
  uint16_t fifoCount = mpu.getFIFOCount();

  if (intStatus & 0x10) {
    mpu.resetFIFO();
    delay(10);
    return;
  }

  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    read_mpu_raw();
    fifoCount = mpu.getFIFOCount();
  }
}

/**
 * @brief Sets the current yaw, pitch, and roll as the reference zero offsets.
 *
 * Stores the current YPR values, converted to degrees, into the global ypr_offset
 * array so subsequent adjusted YPR readings are reported relative to this reference.
 */
void zeroYPR() {
  ypr_offset[0] = ypr[0] * 180.0f / M_PI;
  ypr_offset[1] = ypr[1] * 180.0f / M_PI;
  ypr_offset[2] = ypr[2] * 180.0f / M_PI;
}

/**
 * @brief Produce yaw, pitch, and roll adjusted by the stored zero offsets.
 *
 * Converts the internally-stored YPR values from radians to degrees and subtracts
 * the corresponding saved offsets to produce adjusted angles suitable for display
 * or logging.
 *
 * @param yaw_adj Output yaw angle in degrees (0..360 convention depends on sensor).
 * @param pitch_adj Output pitch angle in degrees.
 * @param roll_adj Output roll angle in degrees.
 */
void get_adjusted_ypr(float &yaw_adj, float &pitch_adj, float &roll_adj) {
  yaw_adj   = (ypr[0] * 180.0f / M_PI) - ypr_offset[0];
  pitch_adj = (ypr[1] * 180.0f / M_PI) - ypr_offset[1];
  roll_adj  = (ypr[2] * 180.0f / M_PI) - ypr_offset[2];
}

/**
 * @brief Map a FlightState value to its human-readable name.
 *
 * @param st Flight state enum value to describe.
 * @return const char* String literal: "GROUND", "FLIGHT", "LANDED", "DONE", or "UNKNOWN" for unrecognized values.
 */
const char* flightStateName(FlightState st) {
  switch (st) {
    case ST_GROUND: return "GROUND";
    case ST_FLIGHT: return "FLIGHT";
    case ST_LANDED: return "LANDED";
    case ST_DONE:   return "DONE";
    default:        return "UNKNOWN";
  }
}

/**
 * @brief Pushes a telemetry sample into the circular pre-launch cache.
 *
 * Stores the provided Sample at the current cache head, advances the head index,
 * and marks the cache as filled when the buffer wraps (older entries will be
 * overwritten once full).
 *
 * @param s Sample to append to the cache (copied into the buffer).
 */
void cachePush(const Sample &s) {
  cacheBuf[cacheHead] = s;
  cacheHead = (cacheHead + 1) % CACHE_SAMPLES;
  if (cacheHead == 0) cacheFilled = true;
}

/**
 * @brief Write the CSV header line to the currently open log file.
 *
 * If no log file is open, this function does nothing. When a file is open it
 * writes the column header: t_ms,h_m,alt_asl_m,vel_mps,p_mbar,t_ms5611_c,
 * t_mpu_c,ax_g,ay_g,az_g,pitch_deg,roll_deg,event
 */
void writeCsvHeader() {
  if (!logFile) return;
  logFile.println("t_ms,h_m,alt_asl_m,vel_mps,p_mbar,t_ms5611_c,t_mpu_c,ax_g,ay_g,az_g,pitch_deg,roll_deg,event");
}

/**
 * @brief Writes a Sample as a CSV row to the currently open log file.
 *
 * If no log file is open this function does nothing. The row contains
 * timestamp, height, altitude ASL, vertical speed, pressure, temperatures,
 * accelerations, pitch and roll, and an optional event tag.
 *
 * @param s Sample data to serialize into the CSV row.
 * @param eventTag Optional null-terminated event tag appended as the last CSV field; pass nullptr or an empty string for no tag.
 */
void writeCsvRow(const Sample &s, const char* eventTag) {
  if (!logFile) return;

  char line[240];
  int n = snprintf(line, sizeof(line),
    "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%s",
    (unsigned long)s.t_ms,
    s.h_m, s.alt_asl_m, s.vel_mps,
    s.p_mbar,
    s.t_ms5611_c, s.t_mpu_c,
    s.ax_g, s.ay_g, s.az_g,
    s.pitch_deg, s.roll_deg,
    (eventTag ? eventTag : "")
  );
  if (n > 0) logFile.println(line);
}

/**
 * @brief Appends any stored pre-launch samples from the in-memory circular cache into the currently open CSV log.
 *
 * If a log file is open, writes every cached Sample to the file in chronological order and tags each row with "CACHE".
 * If no log is open, the function is a no-op.
 */
void dumpCacheToFile() {
  if (!logFile) return;

  uint16_t count = cacheFilled ? CACHE_SAMPLES : cacheHead;
  uint16_t start = cacheFilled ? cacheHead : 0;

  for (uint16_t i = 0; i < count; i++) {
    uint16_t idx = (start + i) % CACHE_SAMPLES;
    writeCsvRow(cacheBuf[idx], "CACHE");
  }
}

/**
 * @brief Ensures a new flight CSV log file is opened and initialized (if not already open).
 *
 * Creates and opens a uniquely-named CSV file on FFat, writes the CSV header, dumps any
 * pre-launch cache into the file, and marks the log as open; no-op if a log is already open.
 *
 * @return true if a log file was already open or was successfully created and initialized, false on failure.
 */
bool openNewLogFile() {
  if (logOpen) return true;

  // Unique-ish without RTC
  uint32_t r = esp_random();
  uint32_t m = millis();
  snprintf(currentLogPath, sizeof(currentLogPath), "/flight_%08lX_%08lX.csv",
           (unsigned long)r, (unsigned long)m);

  logFile = FFat.open(currentLogPath, FILE_WRITE);
  if (!logFile) {
    currentLogPath[0] = 0;
    logOpen = false;
    return false;
  }

  logOpen = true;
  writeCsvHeader();
  dumpCacheToFile();
  logFile.flush();
  return true;
}

/**
 * @brief Flushes and closes the current CSV log file if one is open.
 *
 * If a log is open, this function flushes buffered data to storage, closes the file,
 * and clears the internal open flag. If no log is open, the function returns without action.
 */
void closeLogFileSafe() {
  if (!logOpen) return;
  logFile.flush();
  logFile.close();
  logOpen = false;
}

/**
 * @brief Estimate vertical speed from recent altitude samples.
 *
 * Maintains an internal sliding window of recent altitude/time samples and
 * returns a smoothed vertical velocity in meters per second. Until the window
 * is filled the function returns 0.0.
 *
 * @param hnow Current altitude in meters.
 * @param nowms Current timestamp in milliseconds.
 * @return float Smoothed vertical speed in meters per second.
 */
float computeVerticalSpeed(float hnow, uint32_t nowms) {
  // Push into ring buffer
  hWin[velIdx] = hnow;
  tWin[velIdx] = nowms;
  velIdx = (uint8_t)((velIdx + 1) % VELWIN);
  if (velCount < VELWIN) velCount++;

  // Until window is full, don't output noisy velocity
  if (velCount < VELWIN) {
    lastAvgH = hnow;
    lastAvgT = nowms;
    velfilt = 0.0f;
    return 0.0f;
  }

  // Compute average altitude and average time over the window
  float sumH = 0.0f;
  uint32_t sumT = 0;
  for (uint8_t i = 0; i < VELWIN; i++) {
    sumH += hWin[i];
    sumT += tWin[i];
  }
  const float avgH = sumH / (float)VELWIN;
  const uint32_t avgT = sumT / (uint32_t)VELWIN;

  // Differentiate the averaged signal
  if (lastAvgT == 0) {
    lastAvgT = avgT;
    lastAvgH = avgH;
    return 0.0f;
  }

  uint32_t dtms = avgT - lastAvgT;
  if (dtms < 5) return velfilt; // avoid divide by tiny dt / repeated timestamps

  const float dt = dtms / 1000.0f;
  const float v = (avgH - lastAvgH) / dt;

  // Optional extra smoothing on top of the window (can set VELALPHA=1.0f to disable)
  const float VELALPHA = 0.35f;
  velfilt = VELALPHA * v + (1.0f - VELALPHA) * velfilt;

  lastAvgT = avgT;
  lastAvgH = avgH;
  return velfilt;
}

/**
 * @brief Construct a telemetry Sample populated with the latest sensor and timing values.
 *
 * Populates a Sample with the current timestamp, altitude (AGL and ASL), pressure and temperatures,
 * accelerometer axes in g, pitch and roll in degrees, and the supplied vertical velocity.
 *
 * @param vel_mps_now Vertical speed in meters per second to store in the sample.
 * @return Sample A Sample whose fields contain the most-recent sensor readings; fields with no valid
 * data are set to zero. The timestamp is millisecond uptime. Pitch and roll are expressed in degrees.
 */
Sample makeSample(float vel_mps_now) {
  Sample s{};
  s.t_ms = millis();

  s.h_m = isfinite(height_agl_m) ? (float)height_agl_m : 0.0f;
  s.alt_asl_m = isfinite(altitude_asl_m) ? (float)altitude_asl_m : 0.0f;
  s.vel_mps = vel_mps_now;

  s.p_mbar = isfinite(ms5611_pressure_mbar) ? (float)ms5611_pressure_mbar : 0.0f;
  s.t_ms5611_c = isfinite(ms5611_temp_c) ? (float)ms5611_temp_c : 0.0f;
  s.t_mpu_c = (float)temp_mpu;

  float ax_g, ay_g, az_g;
  convert_accel_to_g(ax_g, ay_g, az_g);
  s.ax_g = ax_g; s.ay_g = ay_g; s.az_g = az_g;

  s.pitch_deg = ypr[1] * 180.0f / M_PI;
  s.roll_deg  = ypr[2] * 180.0f / M_PI;

  return s;
}

/**
 * @brief Update flight state machine and manage CSV logging based on a new telemetry sample.
 *
 * Processes a single Sample to:
 * - maintain the pre-launch circular cache until launch,
 * - detect launch, apogee, and landing events and transition flightState (ST_GROUND → ST_FLIGHT → ST_LANDED → ST_DONE),
 * - open/dump/close CSV logs and write event-tagged rows ("EVENT_LAUNCH", "EVENT_APOGEE", "EVENT_LAND", "POST_LAND", "EVENT_CLOSE"),
 * - periodically flush the open log to reduce data loss,
 * - enable or disable the SoftAP/web server according to flight state and configuration.
 *
 * @param s Current telemetry sample containing timestamp, altitude, vertical speed, and other sensor fields used for detection and logging.
 */
void updateFlightStateAndLogging(const Sample &s) {
  // Always keep a cache until launch
  if (!launchDetected) cachePush(s);

  // Track max height
  if (s.h_m > maxHeight_m) maxHeight_m = s.h_m;

  // ---------------- LAUNCH DETECT ----------------
  if (!launchDetected) {
    bool cond = (s.h_m > LAUNCH_ALT_THRESH_M) && (s.vel_mps > LAUNCH_VEL_THRESH_MPS);
    if (cond) launchConfirm++;
    else if (launchConfirm > 0) launchConfirm--;

    if (launchConfirm >= LAUNCH_CONFIRM_SAMPLES) {
      launchDetected = true;
      flightState = ST_FLIGHT;

      if (openNewLogFile()) {
        writeCsvRow(s, "EVENT_LAUNCH");
        logFile.flush();
      }

      if (WIFI_OFF_DURING_FLIGHT && apMode) {
        stopAPAndServer();
      }
    }
  }

  // ---------------- FLIGHT LOGGING ----------------
  if (flightState == ST_FLIGHT && logOpen) {
    const char* tag = "";

    // APOGEE DETECT (requires descent + drop from max)
    if (!apogeeDetected) {
      bool apCond = (s.vel_mps < APOGEE_VEL_NEG_THRESH_MPS) && ((maxHeight_m - s.h_m) > APOGEE_DROP_THRESH_M);
      if (apCond) apogeeConfirm++;
      else if (apogeeConfirm > 0) apogeeConfirm--;

      if (apogeeConfirm >= APOGEE_CONFIRM_SAMPLES) {
        apogeeDetected = true;
        apogeeAtMs = millis();
        tag = "EVENT_APOGEE";
      }
    }

    writeCsvRow(s, tag);

    // Periodic flush (limits loss, avoids flush every line)
    static uint32_t lastFlushMs = 0;
    uint32_t now = millis();
    if (now - lastFlushMs >= 1000) {
      lastFlushMs = now;
      logFile.flush();
    }
  }

  // ---------------- LANDING DETECT ----------------
  if (flightState == ST_FLIGHT && apogeeDetected && !landingDetected &&
    (millis() - apogeeAtMs) >= LAND_AFTER_APOGEE_DELAY_MS) {
    bool velOk = fabsf(s.vel_mps) < LAND_VEL_ABS_THRESH_MPS;

    if (velOk) {
      if (landStableStartMs == 0) {
        landStableStartMs = s.t_ms;
        landAltRef = s.h_m;
      } else {
        bool altOk = fabsf(s.h_m - landAltRef) < LAND_ALT_RANGE_M;
        if (!altOk) {
          landStableStartMs = s.t_ms;
          landAltRef = s.h_m;
        } else if ((s.t_ms - landStableStartMs) >= LAND_STABLE_MS) {
          landingDetected = true;
          flightState = ST_LANDED;
          landedAtMs = millis();

          if (logOpen) {
            writeCsvRow(s, "EVENT_LAND");
            logFile.flush();
          }
        }
      }
    } else {
      landStableStartMs = 0;
    }
  }

  // ---------------- POST-LANDING WINDOW ----------------
  if (flightState == ST_LANDED) {
    if (logOpen) {
      writeCsvRow(s, "POST_LAND");
    }

    if (millis() - landedAtMs >= POST_LAND_LOG_MS) {
      if (logOpen) {
        writeCsvRow(s, "EVENT_CLOSE");
        closeLogFileSafe();
      }
      flightState = ST_DONE;

      // AP on for recovery/download
      if (!apMode) {
        startAPAndServer();
      }
    }
  }
}

// -------------------- WiFi / Server --------------------

// Optional: fixed AP IP (nice for phones). Comment out if you prefer default 192.168.4.1.
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

/**
 * @brief Start a WiFi SoftAP using configured SSID/password and optional static IP.
 *
 * Attempts to configure a static IP for the AP, sets WiFi mode to AP, and starts a SoftAP
 * on channel 1 with up to 4 clients. If no password is configured the AP is created open.
 *
 * @return true if the SoftAP was started successfully, false otherwise.
 */
static bool startSoftAP() {
  WiFi.mode(WIFI_AP);
  delay(50);

  // Optional static IP
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET); // returns bool; ignore if it fails

  const int channel = 1;
  const int hidden = 0;
  const int maxConn = 4;

  bool ok;
  if (ap_pass == nullptr || strlen(ap_pass) == 0) {
    ok = WiFi.softAP(ap_ssid, nullptr, channel, hidden, maxConn);
  } else {
    ok = WiFi.softAP(ap_ssid, ap_pass, channel, hidden, maxConn);
  }
  return ok;
}

/**
 * @brief Stop the HTTP server and disable the device SoftAP and WiFi radio.
 *
 * Stops the running HTTP server, disconnects and stops the SoftAP (optionally powering off
 * the WiFi radio), and clears the apMode flag to indicate AP is no longer active.
 */
void stopAPAndServer() {
  // Stop HTTP server first
  server.stop();

  // Explicitly stop SoftAP, optionally powering off WiFi radio afterward [web:160]
  WiFi.softAPdisconnect(true); // true => turn WiFi off too
  delay(50);

  WiFi.mode(WIFI_OFF);
  delay(50);

  apMode = false;
}

/**
 * @brief Start the SoftAP and HTTP server and register REST endpoints for telemetry and log management.
 *
 * Starts (or restarts) the SoftAP; if the AP cannot be started, the server is not started. When successful,
 * registers routes for:
 * - "/" : serves the main dashboard page.
 * - "/data" : returns live telemetry JSON.
 * - "/setAltitude" (POST) : set ASL altitude reference.
 * - "/zeroYPR" (POST) : zero current yaw/pitch/roll.
 * - "/reset" (POST) : request device reset.
 * - "/deleteLogs" (POST) : delete all CSV logs.
 * - "/status" : returns a compact JSON status (flight state, event flags, log status/file).
 * - "/logs" : HTML listing of files with links and a delete-all form.
 * - "/download?file=/<path>" : streams the named CSV file as an attachment.
 * - "/latest" : streams the most recent finished flight CSV (prefers closed current log, otherwise picks the largest flight_*.csv).
 *
 * Also sets the server content length to unknown and begins listening for HTTP clients.
 */
void startAPAndServer() {
  // (Re)start AP
  apMode = startSoftAP();

  // If AP start failed, don't start routes/server
  if (!apMode) {
    return;
  }

  // ---- Core routes ----
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setAltitude", HTTP_POST, handleSetAltitude);
  server.on("/zeroYPR", HTTP_POST, handleZeroYPR);
  server.on("/reset", HTTP_POST, handleReset);

  // ---- Delete all CSV logs ----
  server.on("/deleteLogs", HTTP_POST, handleDeleteLogs);

  // ---- Minimal status endpoint ----
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"state\":\"" + String(flightStateName(flightState)) + "\",";
    json += "\"launch\":" + String(launchDetected ? "true" : "false") + ",";
    json += "\"apogee\":" + String(apogeeDetected ? "true" : "false") + ",";
    json += "\"landed\":" + String(landingDetected ? "true" : "false") + ",";
    json += "\"log_open\":" + String(logOpen ? "true" : "false") + ",";
    json += "\"log_file\":\"" + String(currentLogPath) + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  // ---- List logs (simple HTML) ----
  server.on("/logs", HTTP_GET, []() {
    String out;
    out.reserve(3500);
    out += "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head><body>";
    out += "<h3>Logs</h3>";
    out += "<p><a href='/latest'>Download latest</a></p>";
    out += "<form method='POST' action='/deleteLogs' onsubmit='return confirm(\"Delete ALL CSV logs?\")'>"
           "<button type='submit' style='background:#e74c3c;color:white;border:none;padding:10px;border-radius:6px;'>Delete all CSV logs</button>"
           "</form>";
    out += "<ul>";

    File root = FFat.open("/");
    if (!root || !root.isDirectory()) {
      out += "<li>FFat open root failed</li>";
    } else {
      File f = root.openNextFile();
      while (f) {
        String name = String(f.name());
        size_t sz = f.size();
        out += "<li><a href='/download?file=" + name + "'>" + name + "</a> (" + String((unsigned long)sz) + " bytes)</li>";
        f.close();                 // IMPORTANT
        f = root.openNextFile();
      }
      root.close();                // IMPORTANT
    }

    out += "</ul></body></html>";
    server.send(200, "text/html", out);
  });

  // ---- Download: /download?file=/flight_xxx.csv ----
  server.on("/download", HTTP_GET, []() {
    if (!server.hasArg("file")) {
      server.send(400, "text/plain", "Missing file parameter");
      return;
    }
    String path = server.arg("file");
    if (!path.startsWith("/")) path = "/" + path;

    File f = FFat.open(path, "r");
    if (!f) {
      server.send(404, "text/plain", "File not found");
      return;
    }

    String fname = String(f.name());
    if (fname.startsWith("/")) fname = fname.substring(1);

    server.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
    server.streamFile(f, "text/csv");
    f.close();
  });

  // ---- Best-effort: download latest finished log ----
  server.on("/latest", HTTP_GET, []() {
    // Prefer currentLogPath if it exists and is closed
    if (currentLogPath[0] != 0 && !logOpen) {
      File f = FFat.open(currentLogPath, "r");
      if (f) {
        String fname = String(f.name());
        if (fname.startsWith("/")) fname = fname.substring(1);
        server.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
        server.streamFile(f, "text/csv");
        f.close();
        return;
      }
    }

    // Otherwise pick largest /flight_*.csv
    String bestName;
    size_t bestSize = 0;

    File root = FFat.open("/");
    if (root) {
      File f = root.openNextFile();
      while (f) {
        String name = String(f.name());
        size_t sz = f.size();
        if (name.startsWith("/flight_") && name.endsWith(".csv") && sz > bestSize) {
          bestSize = sz;
          bestName = name;
        }
        f = root.openNextFile();
      }
    }

    if (bestName.length() == 0) {
      server.send(404, "text/plain", "No flight logs found");
      return;
    }

    File f = FFat.open(bestName, "r");
    if (!f) {
      server.send(404, "text/plain", "File not found");
      return;
    }

    String fname = bestName;
    if (fname.startsWith("/")) fname = fname.substring(1);
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
    server.streamFile(f, "text/csv");
    f.close();
  });

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.begin();
}

/**
 * @brief Update the status LED to reflect flight and AP state.
 *
 * Sets the LED solid on during flight. After a completed flight it blinks slowly.
 * When not in flight or done, the LED is off if the SoftAP is disabled; if the SoftAP
 * is enabled the LED blinks: slow when one or more clients are connected, fast when none.
 * Blinking is timed non-blockingly using the lastBlinkTime timestamp and the
 * BLINK_FAST / BLINK_SLOW intervals.
 */
void updateLED() {
  unsigned long now = millis();

  if (flightState == ST_FLIGHT) {
    digitalWrite(LED_PIN, HIGH); // solid ON during flight
    return;
  }

  // DONE: slow blink
  if (flightState == ST_DONE) {
    if (now - lastBlinkTime >= BLINK_SLOW) {
      lastBlinkTime = now;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    return;
  }

  // Ground / landed: blink based on client count (only if AP is on)
  if (!apMode) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  int clients = WiFi.softAPgetStationNum();
  unsigned long interval = (clients > 0) ? BLINK_SLOW : BLINK_FAST;
  if (now - lastBlinkTime >= interval) {
    lastBlinkTime = now;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// -------------------- HTML & Server Handlers --------------------

static const char* ALTITUDE_INPUT_DEFAULT = "95.0";

// New handler forward-declare (add to your forward declarations too)
void handleDeleteLogs();

/**
 * @brief Build the device web dashboard HTML page.
 *
 * Returns a complete HTML document (with embedded CSS and JavaScript) used as the web UI
 * for live telemetry and log management. The page polls the /data endpoint, displays
 * sensor values (barometer, MPU6050, orientation, vertical speed), shows flight/log state,
 * and provides controls for setting ASL altitude, zeroing YPR, deleting logs, and resetting
 * the device. The generated HTML embeds the configured AP SSID and the default altitude
 * input value.
 *
 * @return String Full HTML page to be served by the device's HTTP server.
 */
String htmlPage() {
  // NOTE: SSID line uses string concatenation (raw string -> String(ap_ssid) -> raw string)
  String s = R"=====(<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-S3 Sensor Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 10px; background: #f5f5f5; }
    .container { max-width: 800px; margin: 0 auto; }
    .card { background: white; padding: 15px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .sensor-title { color: #2c3e50; border-bottom: 2px solid #3498db; padding-bottom: 4px; margin-bottom: 8px; font-size: 1.1em; }
    .value { font-weight: bold; color: #e74c3c; font-family: 'Courier New', monospace; }
    .unit { color: #7f8c8d; font-size: 0.9em; }
    input[type="number"] { padding: 6px; width: 140px; border: 1px solid #bdc3c7; border-radius: 4px; font-size: 0.9em; }
    button { background: #3498db; color: white; border: none; padding: 8px 12px; border-radius: 4px; cursor: pointer; font-size: 0.9em; margin: 2px; }
    button:hover { background: #2980b9; }
    button:disabled { background: #95a5a6; cursor: not-allowed; }
    .zero-btn { background: #e74c3c; }
    .zero-btn:hover { background: #c0392b; }
    .danger-btn { background: #e74c3c; }
    .danger-btn:hover { background: #c0392b; }
    .timestamp { color: #95a5a6; font-style: italic; font-size: 0.9em; }
    .axis { display: inline-block; width: 110px; margin: 2px 0; }
    .sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .temp-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    @media (max-width: 600px) {
      .sensor-grid { grid-template-columns: 1fr; }
      .temp-grid { grid-template-columns: 1fr; }
    }
    a { color: #3498db; text-decoration: none; }
    .small { color:#7f8c8d; font-size: 0.9em; }
    .statusline { margin-top:6px; }
  </style>

  <script>
    let lastUpdate = 0;
    const UPDATE_INTERVAL = 100;

    function updateData() {
      const now = Date.now();
      if (now - lastUpdate < UPDATE_INTERVAL) return;
      lastUpdate = now;

      fetch('/data')
        .then(r => r.json())
        .then(data => {
            document.getElementById('temp_ms5611').textContent = data.temp_ms5611.toFixed(1);
            document.getElementById('pressure').textContent = data.pressure.toFixed(1);
            document.getElementById('altitude').textContent = data.altitude.toFixed(2);
            document.getElementById('altitude_type').textContent = data.altitude_type;

            // Vertical speed (m/s)
            document.getElementById('vel_mps').textContent = data.vel_mps.toFixed(2);

          document.getElementById('temp_mpu').textContent = data.temp_mpu.toFixed(1);
          document.getElementById('ax_g').textContent = data.ax_g.toFixed(3);
          document.getElementById('ay_g').textContent = data.ay_g.toFixed(3);
          document.getElementById('az_g').textContent = data.az_g.toFixed(3);
          document.getElementById('gx_degs').textContent = data.gx_degs.toFixed(1);
          document.getElementById('gy_degs').textContent = data.gy_degs.toFixed(1);
          document.getElementById('gz_degs').textContent = data.gz_degs.toFixed(1);

          document.getElementById('yaw').textContent = data.yaw.toFixed(1);
          document.getElementById('pitch').textContent = data.pitch.toFixed(1);
          document.getElementById('roll').textContent = data.roll.toFixed(1);

          document.getElementById('state').textContent = data.state;
          document.getElementById('logfile').textContent = data.log_file;

          if (document.getElementById('clients')) {
            document.getElementById('clients').textContent = data.clients;
          }

          // Disable delete if currently logging (safer)
          const delBtn = document.getElementById('deleteBtn');
          if (delBtn) {
            delBtn.disabled = !!data.log_open;
            document.getElementById('deleteHint').textContent =
              data.log_open ? "Disabled while log file is open." : "Deletes all .csv files in FFat.";
          }

          document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
        })
        .catch(_ => {});
    }

    function setAltitude() {
      const altitude = document.getElementById('altitudeInput').value;
      if (!altitude) return;
      fetch('/setAltitude', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'altitude=' + encodeURIComponent(altitude)
      });
    }

    function zeroYPR() {
      fetch('/zeroYPR', { method: 'POST' });
    }

    function deleteLogs() {
      if (!confirm("Delete ALL CSV logs? This cannot be undone.")) return;

      fetch('/deleteLogs', { method: 'POST' })
        .then(r => r.text())
        .then(t => {
          const el = document.getElementById('deleteResult');
          if (el) el.textContent = t;
          // refresh list/status
          updateData();
        })
        .catch(_ => {
          const el = document.getElementById('deleteResult');
          if (el) el.textContent = "Delete request failed.";
        });
    }

    function doReset() {
    if (!confirm("Reset the device now?")) return;
    fetch("reset", { method: "POST" });
    }

    function updateLoop() {
      updateData();
      requestAnimationFrame(updateLoop);
    }

    window.onload = function() {
      // Set default altitude input value (keeps HTML simpler)
      const ai = document.getElementById('altitudeInput');
      if (ai && !ai.value) ai.value = ")=====" + String(ALTITUDE_INPUT_DEFAULT) + R"=====(";
      updateData();
      requestAnimationFrame(updateLoop);
    };
  </script>
</head>

<body>
  <div class="container">
    <h2>ESP32-S3 Sensor Dashboard</h2>
    <p class="timestamp">Last update: <span id="lastUpdate">-</span></p>

    <div class="card">
      <div class="sensor-title">Flight / Logs</div>
      <div>State: <span id="state" class="value">-</span></div>
      <div>Log file: <span id="logfile" class="value">-</span></div>
      <div style="margin-top:8px;">
        <a href="/latest">Download latest</a> |
        <a href="/logs">List all logs</a>
      </div>

      <div class="statusline">
        <button id="deleteBtn" class="danger-btn" onclick="deleteLogs()">Delete all CSV logs</button>
        <button class="danger-btn" onclick="doReset()">Reset device</button>
        <div id="deleteHint" class="small">Deletes all .csv files in FFat.</div>
        <div id="deleteResult" class="small"></div>
      </div>

      <small>After landing, connect to the AP and use these links.</small>
    </div>

    <div class="card">
      <div class="sensor-title">Altitude Reference (ASL)</div>
      <div>
        <input type="number" id="altitudeInput" step="0.1" placeholder="e.g., 95.0" value="95.0">
        <button onclick="setAltitude()">Set ASL Reference</button>
      </div>
      <small>Sets current location ASL altitude; output becomes ASL altitude.</small>
    </div>

    <div class="temp-grid">
        <div class="card">
        <div class="sensor-title">MS5611 Barometer</div>
        <div>Temp: <span id="temp_ms5611" class="value">-</span><span class="unit"> °C</span></div>
        <div>Pressure: <span id="pressure" class="value">-</span><span class="unit"> mbar</span></div>
        <div><span id="altitude_type">Altitude</span>: <span id="altitude" class="value">-</span><span class="unit"> m</span></div>
        <div>Vertical speed: <span id="vel_mps" class="value">-</span><span class="unit"> m/s</span></div>
        </div>

      <div class="card">
        <div class="sensor-title">MPU6050 Temperature</div>
        <div>Temp: <span id="temp_mpu" class="value">-</span><span class="unit"> °C</span></div>
      </div>
    </div>

    <div class="sensor-grid">
      <div class="card">
        <div class="sensor-title">MPU6050 Accelerometer</div>
        <div class="axis">X: <span id="ax_g" class="value">-</span><span class="unit"> g</span></div>
        <div class="axis">Y: <span id="ay_g" class="value">-</span><span class="unit"> g</span></div>
        <div class="axis">Z: <span id="az_g" class="value">-</span><span class="unit"> g</span></div>
      </div>

      <div class="card">
        <div class="sensor-title">MPU6050 Gyroscope</div>
        <div class="axis">X: <span id="gx_degs" class="value">-</span><span class="unit"> °/s</span></div>
        <div class="axis">Y: <span id="gy_degs" class="value">-</span><span class="unit"> °/s</span></div>
        <div class="axis">Z: <span id="gz_degs" class="value">-</span><span class="unit"> °/s</span></div>
      </div>

      <div class="card">
        <div class="sensor-title">MPU6050 Orientation</div>
        <button class="zero-btn" onclick="zeroYPR()">Zero YPR</button>
        <div class="axis">Yaw: <span id="yaw" class="value">-</span><span class="unit">°</span></div>
        <div class="axis">Pitch: <span id="pitch" class="value">-</span><span class="unit">°</span></div>
        <div class="axis">Roll: <span id="roll" class="value">-</span><span class="unit">°</span></div>
      </div>
    </div>

    <div class="card">
      <div class="sensor-title">AP Status</div>
      <div>SSID: <span class="value">)=====" + String(ap_ssid) + R"=====(</span></div>
      <div>Connected clients: <span id="clients" class="value">-</span></div>
    </div>

    <button onclick="updateData()">Refresh Now</button>
  </div>
</body>
</html>
)=====";

  return s;
}

/**
 * @brief Serve the main web dashboard page for the root ("/") HTTP endpoint.
 *
 * Sends the HTML dashboard produced by htmlPage() with a 200 OK status and "text/html" content type.
 */
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

/**
 * @brief Handle HTTP POST to set the altitude above sea level (ASL) reference.
 *
 * Reads the "altitude" request argument (meters) and, using the current MS5611
 * pressure and temperature readings, sets the stored ASL reference via
 * calibrateAltitudeReference().
 *
 * If the "altitude" argument is missing, responds 400 Bad Request. If the
 * MS5611 has no valid pressure/temperature reading available, responds 503
 * Service Unavailable. On success, responds 200 OK with a confirmation message
 * that includes the set altitude (meters, two decimals).
 */
void handleSetAltitude() {
  if (!server.hasArg("altitude")) {
    server.send(400, "text/plain", "Missing altitude parameter");
    return;
  }
  float altitude_asl = server.arg("altitude").toFloat();

  float p = ms5611_pressure_mbar;
  float t = ms5611_temp_c;

  if (!isfinite(p) || !isfinite(t) || p <= 0) {
    server.send(503, "text/plain", "MS5611 not ready yet");
    return;
  }

  calibrateAltitudeReference(p, t, altitude_asl);
  server.send(200, "text/plain", "ASL reference set to " + String(altitude_asl, 2) + " m");
}

/**
 * @brief Set the current yaw/pitch/roll as the zero reference and acknowledge the action to the HTTP client.
 *
 * Updates the device's YPR zero reference so subsequent orientation readings are reported relative to the current attitude.
 * Sends an HTTP 200 response with Content-Type "text/plain" and body "YPR zeroed".
 */
void handleZeroYPR() {
  zeroYPR();
  server.send(200, "text/plain", "YPR zeroed");
}
/**
 * @brief Handle an HTTP request to reboot the device.
 *
 * Sends a 200/plain-text response with the body "Rebooting..." and then
 * restarts the ESP after a short delay.
 */
void handleReset() {
  server.send(200, "text/plain", "Rebooting...");
  delay(150);
  ESP.restart();
}

/**
 * @brief Deletes all CSV log files from the FFat root and reports the result over HTTP.
 *
 * Iterates the FFat root directory, collects filenames that end with ".csv" (case-insensitive),
 * attempts to remove each collected CSV file, and sends an HTTP response describing how many
 * files were deleted, failed, or skipped due to an internal limit.
 *
 * @note If a log file is currently open for writing, the request is refused and an HTTP 409 is sent.
 * @note If the FFat root cannot be opened, an HTTP 500 is sent.
 * @note A fixed upper bound is used when collecting CSV paths; files beyond that bound are counted
 * as skipped and reported in the final response.
 */
void handleDeleteLogs() {
  // Safety: don't delete while actively logging
  if (logOpen) {
    server.send(409, "text/plain", "Refusing: log file currently open");
    return;
  }

  File root = FFat.open("/");
  if (!root || !root.isDirectory()) {
    server.send(500, "text/plain", "FFat open root failed");
    return;
  }

  // Collect first, then delete (more reliable on some FS backends)
  const int MAX_CSV = 64;     // raise if you expect more logs
  String csvPaths[MAX_CSV];
  int found = 0;
  int skipped = 0;

  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String path = String(f.name());

      // Normalize to absolute path
      if (!path.startsWith("/")) path = "/" + path;

      // Case-insensitive ".csv" match
      String lower = path;
      lower.toLowerCase();
      if (lower.endsWith(".csv")) {
        if (found < MAX_CSV) csvPaths[found++] = path;
        else skipped++;
      }
    }
    f = root.openNextFile();
  }
  root.close();

  int deleted = 0;
  int failed = 0;

  for (int i = 0; i < found; i++) {
    if (FFat.remove(csvPaths[i].c_str())) deleted++;
    else failed++;
    yield(); // keep WiFi stack responsive during bulk deletes
  }

  String msg = "Delete complete. Deleted=" + String(deleted) + " Failed=" + String(failed);
  if (skipped > 0) msg += " Skipped=" + String(skipped);

  server.send(200, "text/plain", msg);
}

/**
 * @brief Send a JSON snapshot of current telemetry and system status to the HTTP client.
 *
 * Packages current IMU and barometer values, computed velocity and altitude, orientation,
 * flight/logging state, and connected client count into a JSON object and responds with
 * HTTP 200/ application/json.
 *
 * The JSON includes the following keys: `temp_ms5611`, `temp_mpu`, `pressure`, `altitude`,
 * `altitude_type`, `vel_mps`, `ax_g`, `ay_g`, `az_g`, `gx_degs`, `gy_degs`, `gz_degs`,
 * `yaw`, `pitch`, `roll`, `state`, `log_file`, `log_open`, and `clients`.
 */
void handleData() {
  read_dmp();
  read_mpu_raw();

  float ax_g, ay_g, az_g;
  float gx_degs, gy_degs, gz_degs;
  convert_accel_to_g(ax_g, ay_g, az_g);
  convert_gyro_to_degs(gx_degs, gy_degs, gz_degs);

  float yaw_adj, pitch_adj, roll_adj;
  get_adjusted_ypr(yaw_adj, pitch_adj, roll_adj);

  int clients = apMode ? WiFi.softAPgetStationNum() : 0;

  const float t_ms = isfinite(ms5611_temp_c) ? ms5611_temp_c : 0.0f;
  const float p_ms = isfinite(ms5611_pressure_mbar) ? ms5611_pressure_mbar : 0.0f;
  const float alt  = isfinite(altitude_asl_m) ? altitude_asl_m : 0.0f;

  String alt_label = refSet ? "Altitude ASL" : "Altitude (relative)";

  String json = "{";
  json += "\"temp_ms5611\":" + String(t_ms, 1) + ",";
  json += "\"temp_mpu\":" + String(temp_mpu, 1) + ",";
  json += "\"pressure\":" + String(p_ms, 1) + ",";
  json += "\"altitude\":" + String(alt, 2) + ",";
  json += "\"altitude_type\":\"" + alt_label + "\",";

  // Vertical speed (m/s), computed in loop() at LOG_HZ cadence
  json += "\"vel_mps\":" + String((float)latest_vel_mps, 2) + ",";

  json += "\"ax_g\":" + String(ax_g, 3) + ",";
  json += "\"ay_g\":" + String(ay_g, 3) + ",";
  json += "\"az_g\":" + String(az_g, 3) + ",";

  json += "\"gx_degs\":" + String(gx_degs, 1) + ",";
  json += "\"gy_degs\":" + String(gy_degs, 1) + ",";
  json += "\"gz_degs\":" + String(gz_degs, 1) + ",";

  json += "\"yaw\":" + String(yaw_adj, 1) + ",";
  json += "\"pitch\":" + String(pitch_adj, 1) + ",";
  json += "\"roll\":" + String(roll_adj, 1) + ",";

  json += "\"state\":\"" + String(flightStateName(flightState)) + "\",";
  json += "\"log_file\":\"" + String(currentLogPath) + "\",";
  json += "\"log_open\":" + String(logOpen ? "true" : "false") + ",";
  json += "\"clients\":" + String(clients);

  json += "}";
  server.send(200, "application/json", json);
}

/**
 * @brief Initialize hardware, sensors, filesystem, networking, and logging state.
 *
 * Performs board-wide startup: configures LED and serial, mounts the FFat filesystem
 * (formatting only if mounting fails), initializes the MS5611 barometer and acquires
 * a short set of initial samples to set an altitude reference if possible, configures
 * I2C and initializes the MPU6050 with DMP and calibrated offsets, and starts the
 * soft AP + HTTP server when configured. Also initializes the logging cadence timer.
 */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(300);

  // Mount FFat (format-on-fail)
  if (!FFat.begin(false)) {
    FFat.begin(true);
  }

  // MS5611
  ms5611_begin();

  // Optional: get an initial reference quickly (still uses state machine updates)
  {
    float p_sum = 0.0f, t_sum = 0.0f;
    int good = 0;
    uint32_t start_ms = millis();
    while (millis() - start_ms < 1500 && good < 10) {
      ms5611_task_update();
      if (isfinite(ms5611_pressure_mbar) && isfinite(ms5611_temp_c) && ms5611_pressure_mbar > 0) {
        p_sum += ms5611_pressure_mbar;
        t_sum += ms5611_temp_c;
        good++;
      }
      delay(5);
    }
    if (good > 0) {
      calibrateAltitudeReference(p_sum / good, t_sum / good, 0.0f);
    }
  }

  // ---- MPU6050 DMP (ported from old code behavior) ----
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // Set desired full-scale ranges BEFORE DMP init...
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

  Serial.println(F("Testing MPU6050 connection..."));
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 FAIL");
    while (1) { delay(100); }
  }
  Serial.println("MPU6050 connection successful");

  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  // Apply your calibrated offsets (same constants as before)
  mpu.setXGyroOffset(DEV_GYRO_X_OFFSET);
  mpu.setYGyroOffset(DEV_GYRO_Y_OFFSET);
  mpu.setZGyroOffset(DEV_GYRO_Z_OFFSET);
  mpu.setXAccelOffset(DEV_ACCEL_X_OFFSET);
  mpu.setYAccelOffset(DEV_ACCEL_Y_OFFSET);
  mpu.setZAccelOffset(DEV_ACCEL_Z_OFFSET);

  if (devStatus == 0) {
    Serial.println("Active offsets applied.");

    // IMPORTANT (old code behavior):
    // Re-apply full-scale ranges AFTER dmpInitialize(), because DMP init may change them.
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Optional but helpful after enabling DMP
    mpu.resetFIFO();

    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;

    Serial.println(F("DMP Ready"));
  } else {
    Serial.print("DMP ERROR: ");
    Serial.println(devStatus);
    dmpReady = false;
  }


  // Start AP on ground (optional, can be disabled by WIFI_ON_GROUND=false)
  if (WIFI_ON_GROUND) {
    startAPAndServer();
  } else {
    WiFi.mode(WIFI_OFF);
    apMode = false;
  }

  nextLogMs = millis();
}

/**
 * @brief Main runtime loop: advance sensors, perform periodic logging, serve web requests, and update LED.
 *
 * Repeatedly drives non-blocking sensor work (MS5611 state machine and MPU/DMP reads), enforces the logging cadence
 * (creates a Sample at each LOG_INTERVAL_MS, computes vertical speed, and delegates flight-state transitions and CSV
 * logging), services the HTTP server when SoftAP is active, and updates the status LED. Includes a short delay to
 * yield the scheduler.
 */
void loop() {
  // Keep MS5611 conversions running
  ms5611_task_update();

  // Update IMU
  read_dmp();
  read_mpu_raw();

  // Logging cadence
  const uint32_t now = millis();
  if ((int32_t)(now - nextLogMs) >= 0) {
    nextLogMs += LOG_INTERVAL_MS;

    float h_now = isfinite(height_agl_m) ? (float)height_agl_m : 0.0f;
    float v_now = computeVerticalSpeed(h_now, now);

    // Publish for dashboard without re-running the estimator in handleData()
    latest_vel_mps = v_now;

    Sample s = makeSample(v_now);
    updateFlightStateAndLogging(s);
  }

  // Web server only when AP is on
  if (apMode) {
    server.handleClient();
  }

  updateLED();
  delay(1);
}