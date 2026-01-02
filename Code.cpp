#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"             // NEW: Brownout Library
#include "soc/rtc_cntl_reg.h"    // NEW: Brownout Library

const char* ssid = "WiFi SSID";
const char* password = "password";
String serverName = "192.168.1.x.x";

String serverPath = "/upload";
const int serverPort = 5000;

const int SHELF_DEPTH = 20; 
const int THRESHOLD = 5; 

#define PIR_PIN    13
#define VIB_PIN    2
#define TRIG_PIN   12
#define ECHO_PIN   14
#define BUZZER_PIN 15

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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // FIX: Stop the Brownout Crash
  
  Serial.begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(PIR_PIN, INPUT_PULLDOWN);
  pinMode(VIB_PIN, INPUT_PULLUP);

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
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA; // Lower quality to save power
    config.jpeg_quality = 12;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Try to start camera (it might fail if power is too low)
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera Init Failed: 0x%x", err);
  } else {
    Serial.println("Camera Ready!");
  }

  WiFi.begin(ssid, password);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  Serial.println("\nWiFi Connected");

  uint64_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    
    if (wakeup_pin_mask & (1ULL << VIB_PIN)) {
       Serial.println("WAKEUP: VIBRATION DETECTED!");
       triggerAlarm();
    } 
    else if (wakeup_pin_mask & (1ULL << PIR_PIN)) {
       Serial.println("WAKEUP: MOTION DETECTED!");
       checkShelf();
    }
    else {
      checkShelf();
    }
  } else {
    Serial.println("System Booting... Arming Sensors.");
  }

  uint64_t pinMask = (1ULL << PIR_PIN) | (1ULL << VIB_PIN);
  esp_sleep_enable_ext1_wakeup(pinMask, ESP_EXT1_WAKEUP_ANY_HIGH);
  
  Serial.println("Entering Deep Sleep...");
  Serial.flush(); 
  esp_deep_sleep_start();
}

void loop() {
}

void triggerAlarm() {
    Serial.println("ALARM: SHAKING DETECTED");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000); 
    digitalWrite(BUZZER_PIN, LOW);
    sendPhoto();
}

void checkShelf() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  
  Serial.print("Distance: ");
  Serial.println(distance);

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

void sendPhoto() {
    camera_fb_t * fb = esp_camera_fb_get();
    if(!fb) {
      Serial.println("Camera Capture Failed");
      return;
    }

    String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    String url = "http://" + serverName + ":" + String(serverPort) + serverPath;
    
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

    String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"alert.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--" + boundary + "--\r\n";

    size_t totalLen = head.length() + fb->len + tail.length();
    uint8_t *buffer = (uint8_t *) ps_malloc(totalLen);
    if (!buffer) buffer = (uint8_t *) malloc(totalLen);

    if (buffer) {
       memcpy(buffer, head.c_str(), head.length());
       memcpy(buffer + head.length(), fb->buf, fb->len);
       memcpy(buffer + head.length() + fb->len, tail.c_str(), tail.length());

       int httpResponseCode = http.sendRequest("POST", buffer, totalLen);
       Serial.print("Server Response: ");
       Serial.println(httpResponseCode);
       free(buffer);
    }
    
    esp_camera_fb_return(fb);
    http.end();
}