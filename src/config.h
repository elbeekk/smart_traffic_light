// config.h
#ifndef CONSTANTS_H
#define CONSTANTS_H

// WiFi Credentials
#define WIFI_SSID "TP-Link_CC28"
#define WIFI_PASS "44855185"

// Server Configuration
#define SERVER_URL "http://192.168.0.120:4999/detect"  // Changed from 127.0.0.1 to your network IP
#define LIGHT_STATUS_URL "http://192.168.0.120:4999/light"

// Traffic Light Pins (ESP32-CAM)
#define RED_PIN 12
#define YELLOW_PIN 13
#define GREEN_PIN 14

// Camera Settings
#define FRAME_SIZE FRAMESIZE_QVGA
#define JPEG_QUALITY 10

#endif