#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "ArduinoJson.h"
#include "config.h"
#include "camera.h"

// Debug settings
#define DEBUG_PORT Serial
#define DEBUG_LED 33 // ESP32-CAM built-in LED (active low)

// Traffic light pins
const uint8_t trafficPins[] = {RED_PIN, YELLOW_PIN, GREEN_PIN};

// Timing constants (milliseconds)
const unsigned long CAPTURE_INTERVAL = 5000;  // 5 seconds between captures
const unsigned long GREEN_DURATION = 30000;   // 30s max green time
const unsigned long MIN_GREEN_TIME = 10000;   // 10s minimum green time
const unsigned long YELLOW_TIME = 3000;       // 3s yellow time
const unsigned long MIN_RED_TIME = 5000;      // 5s minimum red time

// System state
enum LightState { RED, YELLOW, GREEN };
LightState currentLight = RED;
unsigned long lastLightChange = 0;
unsigned long lastCaptureTime = 0;
int lastCarCount = 0;
bool cameraInitialized = false;

void blinkLED(uint8_t times = 1, uint16_t duration = 200) {
  for(uint8_t i=0; i<times; i++) {
    digitalWrite(DEBUG_LED, LOW);
    delay(duration);
    digitalWrite(DEBUG_LED, HIGH);
    delay(duration);
  }
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DEBUG_PORT.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);  // Flip vertically if needed
  s->set_hmirror(s, 1); // Flip horizontally if needed
  
  DEBUG_PORT.println("Camera initialized");
  return true;
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  DEBUG_PORT.print("Connecting to WiFi");
  
  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    DEBUG_PORT.print(".");
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PORT.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
    blinkLED(3);
  } else {
    DEBUG_PORT.println("\nWiFi connection failed!");
    blinkLED(10);
    ESP.restart();
  }
}

void setLight(LightState state) {
  if (currentLight == state) return;
  
  // Turn all lights off first
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(trafficPins[i], LOW);
  }
  
  // Set new state
  currentLight = state;
  digitalWrite(trafficPins[state], HIGH);
  lastLightChange = millis();
  
  const char* states[] = {"RED", "YELLOW", "GREEN"};
  DEBUG_PORT.printf("Light changed to %s\n", states[state]);
}

void controlTrafficLights(int carCount) {
  unsigned long currentTime = millis();
  unsigned long stateDuration = currentTime - lastLightChange;
  
  switch(currentLight) {
    case GREEN:
      if (carCount > 0 && stateDuration < GREEN_DURATION) {
        // Extend green light when cars are present
        return;
      }
      if (stateDuration >= MIN_GREEN_TIME) {
        setLight(YELLOW);
      }
      break;
      
    case YELLOW:
      if (stateDuration >= YELLOW_TIME) {
        setLight(RED);
      }
      break;
      
    case RED:
      if ((carCount > 0 && stateDuration >= MIN_RED_TIME) || 
          (carCount == 0 && stateDuration >= MIN_RED_TIME * 2)) {
        setLight(GREEN);
      }
      break;
  }
}

bool captureAndProcessImage() {
  DEBUG_PORT.println("Capturing image...");
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    DEBUG_PORT.println("Capture failed");
    return false;
  }
  
  DEBUG_PORT.printf("Image captured: %d bytes\n", fb->len);
  blinkLED(1, 50); // Quick blink on capture

  // Send to server
  WiFiClient client;
  HTTPClient http;
  http.begin(client, SERVER_URL);
  http.addHeader("Content-Type", "image/jpeg");
  
  int httpCode = http.POST(fb->buf, fb->len);
  bool success = false;
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DEBUG_PORT.println("Server response: " + payload);
    
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      lastCarCount = doc["car_count"] | 0;
      DEBUG_PORT.printf("Detected cars: %d\n", lastCarCount);
      success = true;
    } else {
      DEBUG_PORT.println("JSON parse error");
    }
  } else {
    DEBUG_PORT.printf("HTTP error: %d\n", httpCode);
  }
  
  http.end();
  esp_camera_fb_return(fb);
  return success;
}

void setup() {
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, HIGH); // Turn off LED
  
  DEBUG_PORT.begin(115200);
  delay(1000);
  DEBUG_PORT.println("\nESP32-CAM Smart Traffic Light");
  
  // Initialize traffic lights
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(trafficPins[i], OUTPUT);
    digitalWrite(trafficPins[i], LOW);
  }
  setLight(RED); // Start with red light
  
  // Initialize camera
  cameraInitialized = initCamera();
  
  // Connect to WiFi
  connectWiFi();
  
  DEBUG_PORT.println("System initialized");
  blinkLED(2);
}

void loop() {
  static unsigned long lastHeartbeat = 0;
  unsigned long currentMillis = millis();
  
  // Capture and process image at regular intervals
  if (cameraInitialized && (currentMillis - lastCaptureTime >= CAPTURE_INTERVAL)) {
    if (captureAndProcessImage()) {
      lastCaptureTime = currentMillis;
    } else {
      DEBUG_PORT.println("Waiting 2s before retry...");
      delay(2000);
    }
  }
  
  // Control traffic lights based on last detection
  controlTrafficLights(lastCarCount);
  
  // Heartbeat blink every second
  if (currentMillis - lastHeartbeat >= 1000) {
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
    lastHeartbeat = currentMillis;
  }
  
  delay(10); // Small delay to prevent watchdog triggers
}