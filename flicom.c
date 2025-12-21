#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <math.h>

/* ===================== CONFIGURATION ===================== */
// Hardware
#define MPU_CS 5
#define SD_CS 15

// Timing - matched to BMP280 sampling capability
#define LOOP_INTERVAL_MS 20     // 50Hz to match BMP280 with x16 oversampling
#define SAMPLE_RATE_HZ 50

// Calibration
#define CALIBRATION_SAMPLES 200
#define ACCEL_STILLNESS_THRESHOLD 0.1  // g's - must be still on pad

// Launch detection - CRITICAL: realistic thresholds for low-thrust motors
#define LAUNCH_ACCEL_THRESHOLD 0.5      // g's above gravity (net accel)
#define LAUNCH_ACCEL_SAMPLES 5          // Consecutive samples needed
#define MIN_PAD_STILLNESS_TIME 2000     // ms - must be still before arming

// Burnout detection
#define BURNOUT_ACCEL_THRESHOLD 0.5     // g's - raised for noise immunity
#define BURNOUT_MIN_TIME_MS 500         // Minimum burn time
#define BURNOUT_CONFIRM_SAMPLES 8       // Consecutive low-accel samples

// Apogee detection - altitude peak is PRIMARY, others are supporting
#define APOGEE_VEL_THRESHOLD -0.5       // m/s - slight descent (helper only)
#define APOGEE_ALT_WINDOW 0.3           // m - altitude not increasing (PRIMARY)
#define APOGEE_MIN_ALTITUDE 15.0        // m - minimum AGL for valid apogee
#define APOGEE_CONFIRM_SAMPLES 15       // Samples at 50Hz = 300ms
#define APOGEE_ACCEL_THRESHOLD -0.5     // g's - should see descent accel

// Landing detection
#define LANDING_ALTITUDE_THRESHOLD 3.0  // m AGL
#define LANDING_VELOCITY_THRESHOLD 2.0  // m/s
#define LANDING_CONFIRM_TIME 2000       // ms

// State timeouts (safety)
#define BOOST_TIMEOUT_MS 15000
#define COAST_TIMEOUT_MS 45000

// Filtering
#define ALTITUDE_FILTER_ALPHA 0.65
#define ACCEL_FILTER_ALPHA 0.75
#define VELOCITY_FILTER_ALPHA 0.6

// Data logging
#define LOG_BUFFER_SIZE 512
#define LOG_FLUSH_INTERVAL 500  // ms - NEVER flush during BOOST/COAST

/* ===================== MPU6500 REGISTERS ===================== */
#define WHO_AM_I       0x75
#define PWR_MGMT_1     0x6B
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG2  0x1D
#define CONFIG         0x1A
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41

#define MPU6500_ID     0x70
#define MPU6000_ID     0x68

SPISettings mpuSPI(1000000, MSBFIRST, SPI_MODE0);

/* ===================== BMP280 ===================== */
#define BMP_ADDR 0x76
#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60

/* ===================== FLIGHT STATES ===================== */
typedef enum {
  BOOT,
  SENSOR_INIT,
  CALIBRATING,
  PAD_IDLE,
  ARMED,
  BOOST,
  COAST,
  APOGEE,
  DESCENT,
  LANDED,
  ERROR_STATE
} flight_state_t;

const char* state_names[] = {
  "BOOT", "SENSOR_INIT", "CALIBRATING", "PAD_IDLE", "ARMED", 
  "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "ERROR"
};

flight_state_t state = BOOT;

/* ===================== SENSOR DATA ===================== */
struct SensorData {
  // Raw readings
  float altitude_raw;
  float accel_x_raw, accel_y_raw, accel_z_raw;
  float accel_vert_raw;  // Vertical acceleration (calibrated axis)
  
  // Filtered data
  float altitude_filt;
  float accel_vert_filt;
  
  // Derived data
  float velocity;
  float velocity_filt;
  float accel_magnitude;
  
  // Calibration
  float accel_x_bias, accel_y_bias, accel_z_bias;
  float gravity_magnitude;
  int vertical_axis;      // 0=X, 1=Y, 2=Z
  int vertical_sign;      // 1 or -1
  
  // Health
  bool sensors_valid;
  uint16_t sensor_errors;
  unsigned long last_bmp_read;
  unsigned long last_mpu_read;
};

SensorData data = {0};

/* ===================== STATE TRACKING ===================== */
struct FlightData {
  float ground_altitude;
  float max_altitude;
  float max_velocity;
  float max_accel;
  float apogee_altitude;
  
  unsigned long boot_time;
  unsigned long armed_time;
  unsigned long launch_time;
  unsigned long burnout_time;
  unsigned long apogee_time;
  unsigned long landing_time;
  unsigned long state_enter_time;
  
  // Launch detection
  uint16_t launch_accel_count;
  unsigned long pad_still_start;
  bool pad_is_still;
  
  // Burnout detection
  uint16_t burnout_confirm_count;
  
  // Apogee detection (voting system)
  uint16_t apogee_vote_count;
  float apogee_altitude_window_max;
  bool velocity_negative;
  bool altitude_peaked;
  bool accel_descent;
  
  // Landing detection
  unsigned long landing_detect_start;
  
  // Loop timing
  float last_altitude;
  unsigned long last_update_time;
  float dt;
  
  // Statistics
  uint32_t loop_count;
  uint16_t loop_overruns;
};

FlightData flight = {0};

/* ===================== BMP280 CALIBRATION ===================== */
uint16_t dig_T1; 
int16_t dig_T2, dig_T3;
uint16_t dig_P1; 
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

/* ===================== DATA LOGGING ===================== */
File logFile;
char logBuffer[LOG_BUFFER_SIZE];
uint16_t logBufferPos = 0;
unsigned long lastFlush = 0;
bool sdCardAvailable = false;
char filename[32];

/* ===================== WATCHDOG ===================== */
unsigned long lastLoopTime = 0;
#define WATCHDOG_TIMEOUT_MS 1000

/* ===================== UTILITIES ===================== */
float iir_filter(float prev, float input, float alpha) {
  return alpha * prev + (1.0 - alpha) * input;
}

void enter_state(flight_state_t new_state) {
  state = new_state;
  flight.state_enter_time = millis();
  
  char msg[64];
  snprintf(msg, sizeof(msg), "STATE: %s", state_names[new_state]);
  Serial.println(msg);
  log_message(msg);
}

void enter_error_state(const char* reason) {
  char msg[128];
  snprintf(msg, sizeof(msg), "ERROR: %s", reason);
  Serial.println(msg);
  log_message(msg);
  enter_state(ERROR_STATE);
}

/* ===================== DATA LOGGING ===================== */
void log_message(const char* msg) {
  if (!sdCardAvailable) return;
  
  char line[256];
  snprintf(line, sizeof(line), "%lu,%s\n", millis(), msg);
  
  if (logBufferPos + strlen(line) < LOG_BUFFER_SIZE) {
    strcpy(logBuffer + logBufferPos, line);
    logBufferPos += strlen(line);
  } else {
    flush_log();
    strcpy(logBuffer, line);
    logBufferPos = strlen(line);
  }
}

void log_telemetry() {
  if (!sdCardAvailable) return;
  
  float agl = data.altitude_filt - flight.ground_altitude;
  
  char line[256];
  snprintf(line, sizeof(line), "%lu,DATA,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    millis(),
    state,
    agl,
    data.velocity_filt,
    data.accel_vert_filt,
    data.altitude_raw,
    data.accel_vert_raw
  );
  
  if (logBufferPos + strlen(line) < LOG_BUFFER_SIZE) {
    strcpy(logBuffer + logBufferPos, line);
    logBufferPos += strlen(line);
  } else {
    flush_log();
  }
}

void flush_log() {
  if (!sdCardAvailable || logBufferPos == 0) return;
  
  // CRITICAL: Never flush during critical flight phases
  if (state == BOOST || state == COAST || state == APOGEE) {
    return;  // Defer flush to avoid blocking
  }
  
  if (logFile) {
    logFile.write((uint8_t*)logBuffer, logBufferPos);
    logFile.flush();
    logBufferPos = 0;
  }
}

bool init_sd_card() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card init failed - logging disabled");
    return false;
  }
  
  // Find unique filename
  for (int i = 0; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "/flight%03d.csv", i);
    if (!SD.exists(filename)) break;
  }
  
  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to create log file");
    return false;
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
  
  // Write header
  logFile.println("Time(ms),Type,State,AGL(m),Velocity(m/s),Accel(g),Alt_Raw(m),Accel_Raw(g)");
  logFile.flush();
  
  return true;
}

/* ===================== MPU6500 FUNCTIONS ===================== */
void mpu_write(uint8_t reg, uint8_t data) {
  SPI.beginTransaction(mpuSPI);
  digitalWrite(MPU_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(data);
  digitalWrite(MPU_CS, HIGH);
  SPI.endTransaction();
}

uint8_t mpu_read(uint8_t reg) {
  uint8_t result;
  SPI.beginTransaction(mpuSPI);
  digitalWrite(MPU_CS, LOW);
  SPI.transfer(reg | 0x80);
  result = SPI.transfer(0x00);
  digitalWrite(MPU_CS, HIGH);
  SPI.endTransaction();
  return result;
}

void mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  SPI.beginTransaction(mpuSPI);
  digitalWrite(MPU_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(MPU_CS, HIGH);
  SPI.endTransaction();
}

bool mpu_init() {
  delay(100);  // Let sensor stabilize
  
  uint8_t who = mpu_read(WHO_AM_I);
  if (who != MPU6500_ID && who != MPU6000_ID) {
    Serial.print("MPU WHO_AM_I failed: 0x");
    Serial.println(who, HEX);
    return false;
  }
  
  // Reset device
  mpu_write(PWR_MGMT_1, 0x80);
  delay(100);
  
  // Wake up, use best clock
  mpu_write(PWR_MGMT_1, 0x01);
  delay(10);
  
  // Configure gyro (not used but set anyway)
  mpu_write(GYRO_CONFIG, 0x18);  // ±2000 dps
  
  // Configure accelerometer: ±16g for high-power rockets
  mpu_write(ACCEL_CONFIG, 0x18);  // ±16g range
  
  // Low-pass filter: 92Hz bandwidth
  mpu_write(CONFIG, 0x02);
  mpu_write(ACCEL_CONFIG2, 0x02);
  
  delay(10);
  
  Serial.print("MPU initialized (ID: 0x");
  Serial.print(who, HEX);
  Serial.println(")");
  
  return true;
}

void mpu_read_accel(float *ax, float *ay, float *az) {
  uint8_t buf[6];
  mpu_read_bytes(ACCEL_XOUT_H, buf, 6);
  
  int16_t ax_raw = (buf[0] << 8) | buf[1];
  int16_t ay_raw = (buf[2] << 8) | buf[3];
  int16_t az_raw = (buf[4] << 8) | buf[5];
  
  // ±16g range, 16-bit signed: sensitivity = 2048 LSB/g
  *ax = ax_raw / 2048.0;
  *ay = ay_raw / 2048.0;
  *az = az_raw / 2048.0;
}

/* ===================== BMP280 FUNCTIONS ===================== */
uint8_t bmp_read(uint8_t reg) {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0;
  
  if (Wire.requestFrom(BMP_ADDR, 1) != 1) return 0;
  return Wire.read();
}

void bmp_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP_ADDR, len);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
}

uint32_t bmp_read24(uint8_t reg) {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0;
  
  if (Wire.requestFrom(BMP_ADDR, 3) != 3) return 0;
  return ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
}

bool bmp_init() {
  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  
  uint8_t chip_id = bmp_read(0xD0);
  if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID) {
    Serial.print("BMP280 chip ID failed: 0x");
    Serial.println(chip_id, HEX);
    return false;
  }
  
  // Read calibration
  uint8_t calib[24];
  bmp_read_bytes(0x88, calib, 24);
  
  dig_T1 = calib[0] | (calib[1] << 8);
  dig_T2 = calib[2] | (calib[3] << 8);
  dig_T3 = calib[4] | (calib[5] << 8);
  dig_P1 = calib[6] | (calib[7] << 8);
  dig_P2 = calib[8] | (calib[9] << 8);
  dig_P3 = calib[10] | (calib[11] << 8);
  dig_P4 = calib[12] | (calib[13] << 8);
  dig_P5 = calib[14] | (calib[15] << 8);
  dig_P6 = calib[16] | (calib[17] << 8);
  dig_P7 = calib[18] | (calib[19] << 8);
  dig_P8 = calib[20] | (calib[21] << 8);
  dig_P9 = calib[22] | (calib[23] << 8);
  
  // Reset
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(10);
  
  // Config: temp x2, pressure x16, normal mode
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();
  
  // Filter coefficient 4, standby 0.5ms
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF5);
  Wire.write(0x0C);
  Wire.endTransmission();
  
  delay(100);  // Let it stabilize
  
  Serial.print("BMP280 initialized (ID: 0x");
  Serial.print(chip_id, HEX);
  Serial.println(")");
  
  return true;
}

float bmp_read_altitude() {
  int32_t adc_P = bmp_read24(0xF7) >> 4;
  int32_t adc_T = bmp_read24(0xFA) >> 4;
  
  if (adc_P == 0 || adc_T == 0) return -999.9;
  
  // Temperature
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * dig_T2) >> 11;
  int32_t var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;
  t_fine = var1 + var2;
  
  // Pressure
  int64_t p;
  int64_t v1 = ((int64_t)t_fine) - 128000;
  int64_t v2 = v1 * v1 * dig_P6;
  v2 += ((v1 * dig_P5) << 17);
  v2 += ((int64_t)dig_P4 << 35);
  v1 = ((v1 * v1 * dig_P3) >> 8) + ((v1 * dig_P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1) * dig_P1) >> 33;
  
  if (v1 == 0) return -999.9;
  
  p = 1048576 - adc_P;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  v2 = (dig_P8 * p) >> 19;
  p = ((p + v1 + v2) >> 8) + ((int64_t)dig_P7 << 4);
  
  float pressure = p / 256.0;
  float altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
  
  if (altitude < -500.0 || altitude > 15000.0) return -999.9;
  
  return altitude;
}

/* ===================== CALIBRATION ===================== */
void calibrate_sensors() {
  Serial.println("=== SENSOR CALIBRATION ===");
  Serial.println("Keep rocket STILL and VERTICAL on pad...");
  log_message("Starting calibration");
  
  delay(500);
  
  // Collect samples
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float alt_sum = 0;
  int valid_samples = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az;
    mpu_read_accel(&ax, &ay, &az);
    float alt = bmp_read_altitude();
    
    if (alt > -900.0) {
      ax_sum += ax;
      ay_sum += ay;
      az_sum += az;
      alt_sum += alt;
      valid_samples++;
    }
    
    delay(10);
    
    if (i % 20 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  
  if (valid_samples < CALIBRATION_SAMPLES * 0.8) {
    enter_error_state("Calibration failed - insufficient valid samples");
    return;
  }
  
  // Calculate biases
  data.accel_x_bias = ax_sum / valid_samples;
  data.accel_y_bias = ay_sum / valid_samples;
  data.accel_z_bias = az_sum / valid_samples;
  flight.ground_altitude = alt_sum / valid_samples;
  
  // Determine vertical axis (which axis is closest to ±1g)
  float ax_abs = fabs(data.accel_x_bias);
  float ay_abs = fabs(data.accel_y_bias);
  float az_abs = fabs(data.accel_z_bias);
  
  if (ax_abs > ay_abs && ax_abs > az_abs) {
    data.vertical_axis = 0;
    data.vertical_sign = (data.accel_x_bias > 0) ? 1 : -1;
    data.gravity_magnitude = ax_abs;
  } else if (ay_abs > az_abs) {
    data.vertical_axis = 1;
    data.vertical_sign = (data.accel_y_bias > 0) ? 1 : -1;
    data.gravity_magnitude = ay_abs;
  } else {
    data.vertical_axis = 2;
    data.vertical_sign = (data.accel_z_bias > 0) ? 1 : -1;
    data.gravity_magnitude = az_abs;
  }
  
  // Validate
  if (data.gravity_magnitude < 0.8 || data.gravity_magnitude > 1.2) {
    enter_error_state("Calibration failed - invalid gravity reading");
    return;
  }
  
  // Initialize filters
  data.altitude_filt = flight.ground_altitude;
  flight.last_altitude = flight.ground_altitude;
  data.accel_vert_filt = data.gravity_magnitude;
  
  // Print results
  Serial.println("Calibration complete:");
  Serial.print("  Vertical axis: ");
  Serial.print((data.vertical_axis == 0) ? "X" : (data.vertical_axis == 1) ? "Y" : "Z");
  Serial.print(" (sign: ");
  Serial.print(data.vertical_sign);
  Serial.println(")");
  Serial.print("  Gravity: ");
  Serial.print(data.gravity_magnitude, 3);
  Serial.println(" g");
  Serial.print("  Ground altitude: ");
  Serial.print(flight.ground_altitude, 2);
  Serial.println(" m");
  
  char msg[128];
  snprintf(msg, sizeof(msg), "Calibration: axis=%d sign=%d g=%.3f alt=%.2f", 
    data.vertical_axis, data.vertical_sign, data.gravity_magnitude, flight.ground_altitude);
  log_message(msg);
}

/* ===================== SENSOR UPDATE ===================== */
void update_sensors() {
  unsigned long now = millis();
  flight.dt = (now - flight.last_update_time) / 1000.0;
  flight.last_update_time = now;
  
  // Validate dt
  if (flight.dt < 0.001 || flight.dt > 0.1) {
    flight.dt = 0.01;  // Use expected dt
  }
  
  // Read sensors
  float ax, ay, az;
  mpu_read_accel(&ax, &ay, &az);
  data.altitude_raw = bmp_read_altitude();
  
  // Check validity
  if (data.altitude_raw < -900.0) {
    data.sensors_valid = false;
    data.sensor_errors++;
    return;
  }
  
  data.accel_x_raw = ax;
  data.accel_y_raw = ay;
  data.accel_z_raw = az;
  
  // Extract vertical acceleration (calibrated)
  float accel_on_axis;
  switch(data.vertical_axis) {
    case 0: accel_on_axis = ax; break;
    case 1: accel_on_axis = ay; break;
    case 2: accel_on_axis = az; break;
    default: accel_on_axis = az; break;
  }
  
  // Apply sign and remove gravity to get vertical acceleration
  data.accel_vert_raw = data.vertical_sign * accel_on_axis - data.gravity_magnitude;
  
  // Calculate total acceleration magnitude
  data.accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
  
  // Apply filters
  data.altitude_filt = iir_filter(data.altitude_filt, data.altitude_raw, ALTITUDE_FILTER_ALPHA);
  data.accel_vert_filt = iir_filter(data.accel_vert_filt, data.accel_vert_raw, ACCEL_FILTER_ALPHA);
  
  // Calculate velocity
  data.velocity = (data.altitude_filt - flight.last_altitude) / flight.dt;
  data.velocity_filt = iir_filter(data.velocity_filt, data.velocity, VELOCITY_FILTER_ALPHA);
  
  flight.last_altitude = data.altitude_filt;
  data.sensors_valid = true;
  data.last_mpu_read = now;
  data.last_bmp_read = now;
  
  // Track maximums
  float agl = data.altitude_filt - flight.ground_altitude;
  if (agl > flight.max_altitude) flight.max_altitude = agl;
  if (data.velocity_filt > flight.max_velocity) flight.max_velocity = data.velocity_filt;
  if (data.accel_vert_filt > flight.max_accel) flight.max_accel = data.accel_vert_filt;
}

/* ===================== STATE MACHINE ===================== */
void run_state_machine() {
  unsigned long now = millis();
  float agl = data.altitude_filt - flight.ground_altitude;
  
  switch (state) {
    
    case BOOT:
    case SENSOR_INIT:
    case CALIBRATING:
      // Handled in setup
      break;
    
    case PAD_IDLE:
      // If still for required time, arm
      if (fabsf(data.accel_vert_raw) < ACCEL_STILLNESS_THRESHOLD) {
        if (!flight.pad_is_still) {
          flight.pad_still_start = now;
          flight.pad_is_still = true;
        }
        
        // If still for required time, arm
        if (now - flight.pad_still_start > MIN_PAD_STILLNESS_TIME) {
          flight.armed_time = now;
          enter_state(ARMED);
        }
      } else {
        flight.pad_is_still = false;
      }
      break;
    
    case ARMED:
      // Primary: Acceleration-based launch detect
      if (data.accel_vert_filt > LAUNCH_ACCEL_THRESHOLD) {
        flight.launch_accel_count++;
        if (flight.launch_accel_count >= LAUNCH_ACCEL_SAMPLES) {
          flight.launch_time = now;
          Serial.println("🚀 LAUNCH DETECTED!");
          log_message("LAUNCH DETECTED");
          enter_state(BOOST);
        }
      } else {
        flight.launch_accel_count = 0;
      }
      
      // Disarm with persistence to avoid false triggers from vibration
      {
        static uint8_t move_count = 0;
        if (fabsf(data.accel_vert_filt) > ACCEL_STILLNESS_THRESHOLD * 3) {
          move_count++;
        } else {
          move_count = 0;
        }
        
        if (move_count > 5) {
          Serial.println("Disarmed - rocket moved");
          log_message("DISARMED");
          enter_state(PAD_IDLE);
        }
      }
      break;
    
    case BOOST:
      // Burnout detection: sustained low acceleration
      if (data.accel_vert_filt < BURNOUT_ACCEL_THRESHOLD) {
        flight.burnout_confirm_count++;
        
        if (flight.burnout_confirm_count >= BURNOUT_CONFIRM_SAMPLES) {
          // Also check minimum burn time
          if (now - flight.launch_time > BURNOUT_MIN_TIME_MS) {
            flight.burnout_time = now;
            Serial.println("Motor burnout");
            log_message("BURNOUT");
            enter_state(COAST);
          }
        }
      } else {
        flight.burnout_confirm_count = 0;
      }
      
      // Timeout safety
      if (now - flight.state_enter_time > BOOST_TIMEOUT_MS) {
        Serial.println("Boost timeout");
        log_message("BOOST_TIMEOUT");
        enter_state(COAST);
      }
      break;
    
    case COAST:
      {
        // APOGEE DETECTION: Altitude peak is PRIMARY condition
        // Velocity and acceleration are SUPPORTING evidence only
        
        // Track absolute peak altitude in this window
        if (agl > flight.apogee_altitude_window_max) {
          flight.apogee_altitude_window_max = agl;
          flight.apogee_vote_count = 0;  // Reset if still climbing
        }
        
        // PRIMARY: Altitude has peaked (not increasing beyond threshold)
        bool vote_altitude = (flight.apogee_altitude_window_max - agl) > APOGEE_ALT_WINDOW;
        
        // SUPPORTING: Acceleration shows descent
        bool vote_accel = data.accel_vert_filt < APOGEE_ACCEL_THRESHOLD;
        
        // SUPPORTING: Velocity is negative (helper only, not required)
        bool vote_velocity = data.velocity_filt < APOGEE_VEL_THRESHOLD;
        
        // GATE: Must be above minimum altitude
        bool vote_min_alt = agl > APOGEE_MIN_ALTITUDE;
        
        // Require PRIMARY + at least one supporting condition
        bool apogee_detected = vote_altitude && (vote_accel || vote_velocity) && vote_min_alt;
        
        if (apogee_detected) {
          flight.apogee_vote_count++;
          
          if (flight.apogee_vote_count >= APOGEE_CONFIRM_SAMPLES) {
            flight.apogee_time = now;
            flight.apogee_altitude = flight.apogee_altitude_window_max;  // Use peak, not current
            Serial.print("🎯 APOGEE! Altitude: ");
            Serial.print(flight.apogee_altitude, 2);
            Serial.println(" m");
            log_message("APOGEE DETECTED");
            enter_state(APOGEE);
          }
        } else {
          if (flight.apogee_vote_count > 0) {
            flight.apogee_vote_count--;  // Decay counter
          }
        }
        
        // Timeout safety
        if (now - flight.state_enter_time > COAST_TIMEOUT_MS) {
          Serial.println("Coast timeout");
          log_message("COAST_TIMEOUT");
          flight.apogee_altitude = flight.max_altitude;
          enter_state(DESCENT);
        }
      }
      break;
    
    case APOGEE:
      // Transition immediately to descent
      // This state exists for deployment event timing
      enter_state(DESCENT);
      break;
    
    case DESCENT:
      // Landing detection: low altitude AND low velocity for sustained time
      if (agl < LANDING_ALTITUDE_THRESHOLD && fabsf(data.velocity_filt) < LANDING_VELOCITY_THRESHOLD) {
        if (flight.landing_detect_start == 0) {
          flight.landing_detect_start = now;
        }
        
        if (now - flight.landing_detect_start > LANDING_CONFIRM_TIME) {
          flight.landing_time = now;
          Serial.println("🎉 LANDED!");
          log_message("LANDED");
          print_flight_summary();
          
          // Force flush all logs on landing
          flush_log();
          
          enter_state(LANDED);
        }
      } else {
        flight.landing_detect_start = 0;
      }
      break;
    
    case LANDED:
      // Flush logs periodically and final flush
      {
        static unsigned long last_landed_flush = 0;
        if (now - last_landed_flush > 5000) {
          flush_log();
          last_landed_flush = now;
        }
      }
      break;
    
    case ERROR_STATE:
      // Stay in error
      break;
  }
}

/* ===================== TELEMETRY ===================== */
void print_telemetry() {
  static unsigned long last_print = 0;
  unsigned long now = millis();
  
  // Adjust rate based on state
  unsigned long interval;
  switch(state) {
    case PAD_IDLE:
    case ARMED:
      interval = 500;  // 2 Hz
      break;
    case LANDED:
      interval = 2000;  // 0.5 Hz
      break;
    default:
      interval = 100;  // 10 Hz during flight
      break;
  }
  
  if (now - last_print < interval) return;
  last_print = now;
  
  float agl = data.altitude_filt - flight.ground_altitude;
  
  // Console output
  Serial.print(now / 1000.0, 2);
  Serial.print(" | ");
  Serial.print(state_names[state]);
  Serial.print(" | AGL:");
  Serial.print(agl, 1);
  Serial.print("m V:");
  Serial.print(data.velocity_filt, 1);
  Serial.print("m/s A:");
  Serial.print(data.accel_vert_filt, 2);
  Serial.println("g");
  
  // Log to SD
  log_telemetry();
}

void print_flight_summary() {
  Serial.println("\n========== FLIGHT SUMMARY ==========");
  Serial.print("Max altitude:     ");
  Serial.print(flight.max_altitude, 2);
  Serial.println(" m");
  Serial.print("Max velocity:     ");
  Serial.print(flight.max_velocity, 2);
  Serial.println(" m/s");
  Serial.print("Max acceleration: ");
  Serial.print(flight.max_accel, 2);
  Serial.println(" g");
  
  if (flight.burnout_time > 0) {
    Serial.print("Burn time:        ");
    Serial.print((flight.burnout_time - flight.launch_time) / 1000.0, 2);
    Serial.println(" s");
  }
  
  if (flight.apogee_time > 0) {
    Serial.print("Time to apogee:   ");
    Serial.print((flight.apogee_time - flight.launch_time) / 1000.0, 2);
    Serial.println(" s");
  }
  
  if (flight.landing_time > 0) {
    Serial.print("Flight time:      ");
    Serial.print((flight.landing_time - flight.launch_time) / 1000.0, 2);
    Serial.println(" s");
  }
  
  Serial.print("Loop overruns:    ");
  Serial.println(flight.loop_overruns);
  Serial.print("Sensor errors:    ");
  Serial.println(data.sensor_errors);
  Serial.println("====================================\n");
  
  char summary[512];
  snprintf(summary, sizeof(summary), 
    "SUMMARY: Alt=%.2fm Vel=%.2fm/s Accel=%.2fg Burn=%.2fs ToApogee=%.2fs Flight=%.2fs",
    flight.max_altitude, flight.max_velocity, flight.max_accel,
    (flight.burnout_time - flight.launch_time) / 1000.0,
    (flight.apogee_time - flight.launch_time) / 1000.0,
    (flight.landing_time - flight.launch_time) / 1000.0
  );
  log_message(summary);
}

/* ===================== WATCHDOG ===================== */
void check_watchdog() {
  unsigned long now = millis();
  if (now - lastLoopTime > WATCHDOG_TIMEOUT_MS) {
    enter_error_state("Watchdog timeout - loop hang detected");
  }
  lastLoopTime = now;
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  ROCKET FLIGHT COMPUTER v2.0      ║");
  Serial.println("║  Flight-Ready Edition              ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  flight.boot_time = millis();
  enter_state(BOOT);
  
  // Initialize SD card first
  Serial.print("Initializing SD card... ");
  sdCardAvailable = init_sd_card();
  if (sdCardAvailable) {
    Serial.println("OK");
  }
  
  // Initialize SPI for MPU
  pinMode(MPU_CS, OUTPUT);
  digitalWrite(MPU_CS, HIGH);
  SPI.begin(18, 19, 23, MPU_CS);
  
  enter_state(SENSOR_INIT);
  
  // Initialize sensors
  Serial.print("Initializing MPU6500... ");
  if (!mpu_init()) {
    enter_error_state("MPU6500 init failed");
    return;
  }
  Serial.println("OK");
  
  Serial.print("Initializing BMP280... ");
  if (!bmp_init()) {
    enter_error_state("BMP280 init failed");
    return;
  }
  Serial.println("OK");
  
  // Calibrate
  enter_state(CALIBRATING);
  calibrate_sensors();
  
  if (state == ERROR_STATE) return;
  
  // Ready
  flight.last_update_time = millis();
  lastLoopTime = millis();
  enter_state(PAD_IDLE);
  
  Serial.println("\n✓ System ready");
  Serial.println("Keep rocket still for 2 seconds to ARM");
  Serial.println("Logging to: " + String(filename));
  Serial.println("\nTime | State | AGL | Velocity | Accel\n");
}

/* ===================== MAIN LOOP ===================== */
void loop() {
  static unsigned long last_loop = 0;
  unsigned long now = millis();
  
  // Fixed interval timing
  if (now - last_loop < LOOP_INTERVAL_MS) {
    return;
  }
  
  // Check for overruns
  if (now - last_loop > LOOP_INTERVAL_MS * 2) {
    flight.loop_overruns++;
  }
  
  last_loop = now;
  flight.loop_count++;
  
  // Watchdog
  check_watchdog();
  
  if (state == ERROR_STATE) {
    delay(1000);
    return;
  }
  
  // Update sensors
  update_sensors();
  
  if (!data.sensors_valid) {
    if (data.sensor_errors < 10) {
      Serial.println("⚠ Sensor read error");
    }
    if (data.sensor_errors > 50) {
      enter_error_state("Too many sensor errors");
    }
    return;
  }
  
  // Run state machine
  run_state_machine();
  
  // Print telemetry
  print_telemetry();
  
  // Flush logs periodically (but NOT during critical flight phases)
  if (sdCardAvailable && now - lastFlush > LOG_FLUSH_INTERVAL) {
    flush_log();  // flush_log() internally checks for critical states
    lastFlush = now;
  }
}
