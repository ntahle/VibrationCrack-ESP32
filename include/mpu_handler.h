#ifndef MPU_HANDLER_H
#define MPU_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// MPU6050 I2C address and registers
#define MPU_ADDR            0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define ACCEL_SENSITIVITY   16384.0f

// Advanced vibration processing
#define MPU_SAMPLE_US       10000UL  // Sample every 10ms (100 Hz)
#define RMS_WINDOW          100      // 100 samples for RMS
#define GRAVITY_ALPHA       0.995f   // High-pass filter coefficient

// Flex sensor accurate calibration
#define VCC_FLEX            3.3f
#define R_FIXED             10000.0f
#define MAX_FLEX_MM         50.0f
#define FLEX_SAMPLES        10

// Calibration for sensor 1 (GPIO 39)
#define R1_MIN              80000.0f
#define R1_MAX              65000.0f

// Calibration for sensor 2 (GPIO 36)
#define R2_MIN              140000.0f
#define R2_MAX              80000.0f

extern Adafruit_SSD1306 display;
extern WiFiClient espClient;
extern PubSubClient client;

extern const char* ssid;
extern const char* password;

/* MQTT */
extern const char* mqtt_server;
extern const char* mqtt_username;
extern const char* mqtt_password;
extern const char* topic_vibration;

/* PINS */


/* THRESHOLDS */
extern float vibrationThreshold;
extern float flexThreshold;

/* VARIABLES */
extern float flex1_mm;
extern float flex2_mm;
extern float expansionInc1;
extern float expansionInc2;
extern float prevFlex1;
extern float prevFlex2;

// MPU6050 advanced processing state
extern float rms_buffer[RMS_WINDOW];
extern int rms_idx;
extern float rms_sum_sq;
extern float peak_in_window;
extern float grav_x, grav_y, grav_z;
extern unsigned long lastMPUus;


void scanI2C() {
  Serial.println("\n=== Scanning I2C bus ===");
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address: 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.printf("Found %d device(s)\n", count);
  }
  Serial.println("===================\n");
}

// Write to MPU6050 register
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read multiple bytes from MPU6050
void mpuReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

// Initialize MPU6050
void mpuInit() {
  Serial.println("Initializing MPU6050...");
  mpuWrite(REG_PWR_MGMT_1,   0x00);  // Wake up
  delay(100);
  mpuWrite(REG_SMPLRT_DIV,   0x00);  // Sample rate = 1kHz
  mpuWrite(REG_CONFIG,       0x03);  // DLPF 44 Hz
  mpuWrite(REG_GYRO_CONFIG,  0x00);  // Gyro ±250 °/s
  mpuWrite(REG_ACCEL_CONFIG, 0x00);  // Accel ±2g
  memset(rms_buffer, 0, sizeof(rms_buffer));
  rms_sum_sq = 0.0f;
  peak_in_window = 0.0f;
  grav_x = grav_y = grav_z = 0.0f;
  rms_idx = 0;
  Serial.println("MPU6050 initialized!");
}

/* WIFI SETUP */
void setup_wifi()
{
  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

/* MQTT RECONNECT */
void reconnect()
{
  if(client.connected())
    return;

  Serial.print("Connecting MQTT...");

  String clientId = "ESP32_STRUCT_MONITOR-";
  clientId += String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF), HEX);

  // Empty username/password works for public HiveMQ, but keep parameters for compatibility.
  if(client.connect(clientId.c_str(), mqtt_username, mqtt_password))
  {
    Serial.println("connected");
  }
  else
  {
    Serial.print("failed rc=");
    Serial.println(client.state());
  }
}

/* ACCURATE FLEX SENSOR READING */

// Get averaged resistance from flex sensor using voltage divider
float getFlexResistance(int pin)
{
  long totalADC = 0;

  for (int i = 0; i < FLEX_SAMPLES; i++) {
    totalADC += analogRead(pin);
    delayMicroseconds(500);
  }

  float adcValue = totalADC / (float)FLEX_SAMPLES;
  float vOut = (adcValue / 4095.0f) * VCC_FLEX;

  // Avoid division by zero
  if (VCC_FLEX - vOut <= 0.01f) return 0.0f;

  return R_FIXED * (vOut / (VCC_FLEX - vOut));
}

// Convert resistance to displacement (mm) based on calibration
float resistanceToMM(float resistance, float rMin, float rMax)
{
  float mm = (resistance - rMin) * MAX_FLEX_MM / (rMax - rMin);

  // Clamp to valid range
  if (mm < 0.0f) mm = 0.0f;
  if (mm > MAX_FLEX_MM) mm = MAX_FLEX_MM;

  return mm;
}

// Read flex sensor with proper calibration
float readFlex(int pin)
{
  float resistance = getFlexResistance(pin);
  
  // Select calibration based on pin
  if (pin == 39) {
    return resistanceToMM(resistance, R1_MIN, R1_MAX);
  } else if (pin == 36) {
    return resistanceToMM(resistance, R2_MIN, R2_MAX);
  } else {
    // Fallback to sensor 1 calibration
    return resistanceToMM(resistance, R1_MIN, R1_MAX);
  }
}

/* ACCURATE VIBRATION PROCESSING - FYP Method */
void processMPUSample()
{
  // Read accelerometer (6 bytes)
  uint8_t accel_data[6];
  mpuReadBytes(REG_ACCEL_XOUT_H, accel_data, 6);
  
  int16_t ax_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  int16_t ay_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  int16_t az_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);
  
  float ax_g = ax_raw / ACCEL_SENSITIVITY;
  float ay_g = ay_raw / ACCEL_SENSITIVITY;
  float az_g = az_raw / ACCEL_SENSITIVITY;

  // Convert from g to m/s²
  float ax_ms2 = ax_g * 9.81f;
  float ay_ms2 = ay_g * 9.81f;
  float az_ms2 = az_g * 9.81f;

  // High-pass filter to remove gravity (per-axis adaptive)
  grav_x = GRAVITY_ALPHA * grav_x + (1.0f - GRAVITY_ALPHA) * ax_ms2;
  grav_y = GRAVITY_ALPHA * grav_y + (1.0f - GRAVITY_ALPHA) * ay_ms2;
  grav_z = GRAVITY_ALPHA * grav_z + (1.0f - GRAVITY_ALPHA) * az_ms2;

  float dx = ax_ms2 - grav_x;
  float dy = ay_ms2 - grav_y;
  float dz = az_ms2 - grav_z;

  float mag = sqrtf(dx*dx + dy*dy + dz*dz);

  // Rolling RMS window
  rms_sum_sq -= rms_buffer[rms_idx] * rms_buffer[rms_idx];
  rms_buffer[rms_idx] = mag;
  rms_sum_sq += mag * mag;
  rms_idx = (rms_idx + 1) % RMS_WINDOW;

  // Track peak acceleration
  if (mag > peak_in_window) peak_in_window = mag;
}

// Get current RMS acceleration
float getRMS()
{
  return sqrtf(rms_sum_sq / RMS_WINDOW);
}

// Get and reset peak acceleration
float getPeakAndReset()
{
  float peak = peak_in_window;
  peak_in_window = 0.0f;
  return peak;
}

/* OLED NORMAL DISPLAY */
void displayNormal(float vibration)
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print("Vibration:");
  display.print(vibration, 3);
  display.println(" m/s2");

  display.print("Flex1:");
  display.print(expansionInc1, 2);
  display.println(" mm");

  display.print("Flex2:");
  display.print(expansionInc2, 2);
  display.println(" mm");

  display.println();
  display.println("STATUS: NORMAL");

  display.display();
}

/* OLED ALERT DISPLAY */
void displayAlert(float vibration)
{
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.println("WARNING!");

  display.setTextSize(1);
  display.println("Crack Expansion");

  display.print("Vib:");
  display.print(vibration, 3);
  display.println(" m/s2");

  display.print("F1:");
  display.print(expansionInc1, 2);
  display.println(" mm");

  display.print("F2:");
  display.print(expansionInc2, 2);
  display.println(" mm");

  display.display();
}

#endif  