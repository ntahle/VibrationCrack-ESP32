#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "mpu_handler.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define FLEX1 39
#define FLEX2 36
#define BUZZER 25
#define LED 26

/* WIFI */
const char* ssid = "pengulang";
const char* password = "1234abcd";

/* MQTT - Node-RED Broker */
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* topic_vibration = "structural_monitor/data";

// /* PINS */
// #define FLEX1 39
// #define FLEX2 36
// #define BUZZER 25
// #define LED 26

/* THRESHOLDS */
float vibrationThreshold = 1;  // Peak threshold (m/s²) - was 0.12g
float flexThreshold = 5;

/* VARIABLES */
float flex1_mm = 0;
float flex2_mm = 0;
float expansionInc1 = 0;
float expansionInc2 = 0;
float prevFlex1 = 0;
float prevFlex2 = 0;

// MPU6050 advanced processing state
float rms_buffer[RMS_WINDOW];
int rms_idx = 0;
float rms_sum_sq = 0.0f;
float peak_in_window = 0.0f;
float grav_x = 0.0f, grav_y = 0.0f, grav_z = 0.0f;
unsigned long lastMPUus = 0;
unsigned long lastDisplayMs = 0;

/* MQTT */
WiFiClient espClient;
PubSubClient client(espClient);

/* SETUP */
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ESP32 Structural Monitor ===");

  // Initialize ADC for flex sensors
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Full range 0-3.3V
  
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(BUZZER, LOW);
  digitalWrite(LED, LOW);

  // Initialize I2C
  Wire.begin(21, 22);  // SDA=21, SCL=22
  Serial.println("I2C initialized on GPIO21(SDA) and GPIO22(SCL)");
  
  delay(100);

  // Scan I2C bus
  scanI2C();

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED not found!");
  }
  else
  {
    Serial.println("OLED initialized!");
    display.clearDisplay();
    display.display();
  }

  // Initialize MPU6050
  mpuInit();

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  Serial.println("System Ready\n");
}

/* LOOP */
void loop()
{
  // MQTT keep-alive
  if(!client.connected())
    reconnect();

  client.loop();

  // High-rate MPU6050 sampling (every 10ms = 100 Hz)
  unsigned long nowUs = micros();
  if (nowUs - lastMPUus >= MPU_SAMPLE_US) {
    lastMPUus = nowUs;
    processMPUSample();
  }

    // Slower update for flex sensors, OLED (every 1000ms)
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs >= 1000) {
    lastDisplayMs = nowMs;

    flex1_mm = readFlex(FLEX1);
    flex2_mm = readFlex(FLEX2);

    // Get peak acceleration from the last second
    float peakAccel = getPeakAndReset();
    float rmsAccel = getRMS();

    float rawExpansion1 = flex1_mm - prevFlex1;
    float rawExpansion2 = flex2_mm - prevFlex2;

    // Only count as expansion when increase is greater than threshold (5mm by default).
    expansionInc1 = (rawExpansion1 > flexThreshold) ? rawExpansion1 : 0.0f;
    expansionInc2 = (rawExpansion2 > flexThreshold) ? rawExpansion2 : 0.0f;


    bool flexEvent = (expansionInc1 > 0.0f) || (expansionInc2 > 0.0f);
    bool vibEvent = peakAccel >= vibrationThreshold;

    /* ALERT LOGIC */
    if(flexEvent && vibEvent)
    {
      digitalWrite(BUZZER, HIGH);
      digitalWrite(LED, HIGH);

      displayAlert(peakAccel);

      Serial.println("ALERT: Crack expansion under vibration");
    }
    else
    {
      digitalWrite(BUZZER, LOW);
      digitalWrite(LED, LOW);

      displayNormal(peakAccel);
    }
    char jsonPayload[256];
    snprintf(jsonPayload, sizeof(jsonPayload), 
      "{\"peakAccel\":%.4f,\"rmsAccel\":%.4f,\"flex1_mm\":%.2f,\"flex2_mm\":%.2f,\"prevFlex1\":%.2f,\"prevFlex2\":%.2f,\"expansion1\":%.2f,\"expansion2\":%.2f,\"alert\":%s}",
      peakAccel, rmsAccel, flex1_mm, flex2_mm, prevFlex1, prevFlex2, expansionInc1, expansionInc2, flexEvent && vibEvent ? "1" : "0");
    bool ok = client.publish(topic_vibration, jsonPayload);
    if (!ok) {
      Serial.print("MQTT publish failed, state=");
      Serial.println(client.state());
    }

    Serial.print("Peak:");
    Serial.print(peakAccel, 4);
    Serial.print("m/s² | RMS:");
    Serial.print(rmsAccel, 4);
    Serial.print("m/s² | F1:");
    Serial.print(flex1_mm, 2);
    Serial.print("mm | F2:");
    Serial.print(flex2_mm, 2);
    Serial.print("mm | dF1:");
    Serial.print(expansionInc1, 2);
    Serial.print("mm | dF2:");
    Serial.print(expansionInc2, 2);
    Serial.println("mm");

    prevFlex1 = flex1_mm;
    prevFlex2 = flex2_mm;
  }
}