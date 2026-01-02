/*
 * PROJECT: IoT Smart Shelf Anti-Theft System
 * HARDWARE: ESP32-CAM (AI-Thinker), Ultrasonic Sensor, Vibration Sensor, Buzzer
 * FEATURES: Deep Sleep (Power Saving), False Alarm Filtering, WiFi Image Upload
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"           // Library to access internal power registers
#include "soc/rtc_cntl_reg.h"  // Library to control brownout detector

// --- NETWORK CONFIGURATION ---
const char* ssid = "WiFi SSID";        // YOUR WIFI NAME
const char* password = "password";     // YOUR WIFI PASSWORD
// IMPORTANT: Update this IP to your laptop's current IP address from 'ipconfig'
String serverName = "192.168.1.x.x";   

String serverPath = "/upload";         // The API route on the Python Server
const int serverPort = 5000;           // The port the Python Server listens on

// --- SECURITY LOGIC CONSTANTS ---
const int SHELF_DEPTH = 20;  // The normal distance to the item (in cm)
const int THRESHOLD = 5;     // Margin of error to prevent false alarms

// --- PIN DEFINITIONS ---
#define PIR_PIN    13  // Motion Sensor (Detects humans)
#define VIB_PIN    2   // Vibration Sensor (Detects force/impact)
#define TRIG_PIN   12  // Ultrasonic Trigger (Sends sound)
#define ECHO_PIN   14  // Ultrasonic Echo (Receives sound)
#define BUZZER_PIN 15  // Active Buzzer (Alarm)

// --- CAMERA HARDWARE PINOUT (AI-Thinker Model) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setup() {
  // 1. HARDWARE FIX: Disable "Brownout Detector"
  // This prevents the ESP32 from restarting if the camera pulls too much power instantly.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  
  Serial.begin(115200);
  
  // 2. PIN CONFIGURATION
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure alarm is OFF at startup

  // PULLDOWN ensures PIR signal stays LOW (0) until it detects motion (1)
  pinMode(PIR_PIN, INPUT_PULLDOWN);
  // PULLUP keeps Vibration signal HIGH (1) until sensor shakes and connects to GND (0)
  pinMode(VIB_PIN, INPUT_PULLUP);

  // 3. CAMERA CONFIGURATION
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // JPEG is smaller and faster for WiFi upload
  
  // Memory check: Use High Quality if RAM allows, otherwise lower it
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA; 
    config.jpeg_quality = 12; // 0-63, lower number means higher quality
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Initialize Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera Init Failed: 0x%x", err);
  } else {
    Serial.println("Camera Ready!");
  }

  // 4. WIFI CONNECTION
  // We need WiFi to send the photo evidence to the server
  WiFi.begin(ssid, password);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  Serial.println("\nWiFi Connected");

  // 5. DEEP SLEEP LOGIC (The Smart Part)
  // Check WHY the ESP32 turned on. Was it the reset button? Or a sensor?
  uint64_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    // If we woke up because of a sensor, find out WHICH one.
    uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    
    if (wakeup_pin_mask & (1ULL << VIB_PIN)) {
       // SCENARIO 1: The shelf was shaken violently
       Serial.println("WAKEUP: VIBRATION DETECTED!");
       triggerAlarm();
    } 
    else if (wakeup_pin_mask & (1ULL << PIR_PIN)) {
       // SCENARIO 2: A human hand was seen
       Serial.println("WAKEUP: MOTION DETECTED!");
       checkShelf(); // Double-check with Ultrasonic before alarming
    }
    else {
       checkShelf(); // Default safety check
    }
  } else {
    // SCENARIO 3: Normal Startup (Reset Button)
    Serial.println("System Booting... Arming Sensors.");
  }

  // 6. GO TO SLEEP
  // Configure the pins that are allowed to wake the ESP32 up
  uint64_t pinMask = (1ULL << PIR_PIN) | (1ULL << VIB_PIN);
  // ESP_EXT1_WAKEUP_ANY_HIGH means "Wake up if ANY of these pins goes HIGH"
  esp_sleep_enable_ext1_wakeup(pinMask, ESP_EXT1_WAKEUP_ANY_HIGH);
  
  Serial.println("Entering Deep Sleep...");
  Serial.flush(); 
  esp_deep_sleep_start(); // SHUT DOWN CPU TO SAVE BATTERY
}

// Loop is empty because Deep Sleep restarts the device completely.
// It never reaches the loop.
void loop() {
}

// --- FUNCTION: IMMEDIATE ALARM ---
// Used when vibration is detected (Active Theft)
void triggerAlarm() {
    Serial.println("ALARM: SHAKING DETECTED");
    digitalWrite(BUZZER_PIN, HIGH); // Sound Alarm
    delay(1000); 
    digitalWrite(BUZZER_PIN, LOW);  // Silence
    sendPhoto();                    // Upload evidence
}

// --- FUNCTION: VERIFY THEFT ---
// Used when motion is detected (Check if item is actually gone)
void checkShelf() {
  // Send Ultrasonic Pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read Echo
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2; // Convert speed of sound to cm
  
  Serial.print("Distance: ");
  Serial.println(distance);

  // LOGIC: If distance is GREATER than shelf depth, the item is GONE.
  if (distance > (SHELF_DEPTH + THRESHOLD)) {
    Serial.println("THEFT! Sending Alert...");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    sendPhoto();
  } else {
    Serial.println("False Alarm. Item still there.");
  }
}

// --- FUNCTION: UPLOAD EVIDENCE ---
// Takes a photo and sends it to the Python Server via HTTP POST
void sendPhoto() {
    camera_fb_t * fb = esp_camera_fb_get(); // Take Picture
    if(!fb) {
      Serial.println("Camera Capture Failed");
      return;
    }

    // Prepare HTTP Request with Multipart Data (Standard for file uploads)
    String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    String url = "http://" + serverName + ":" + String(serverPort) + serverPath;
    
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

    // Construct the Header and Footer for the image data
    String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"alert.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--" + boundary + "--\r\n";

    // Combine Header + Image + Footer
    size_t totalLen = head.length() + fb->len + tail.length();
    uint8_t *buffer = (uint8_t *) ps_malloc(totalLen);
    if (!buffer) buffer = (uint8_t *) malloc(totalLen);

    if (buffer) {
       memcpy(buffer, head.c_str(), head.length());
       memcpy(buffer + head.length(), fb->buf, fb->len);
       memcpy(buffer + head.length() + fb->len, tail.c_str(), tail.length());

       // Send the POST request
       int httpResponseCode = http.sendRequest("POST", buffer, totalLen);
       Serial.print("Server Response: ");
       Serial.println(httpResponseCode); // 200 = Success, -1 = Error
       free(buffer);
    }
    
    esp_camera_fb_return(fb); // Free camera memory
    http.end(); // Close connection
}