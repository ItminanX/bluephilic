/*
  esp32_cam_client.ino
  ESP32-S3 N16R8 + OV2640 -> POSTs a JPEG frame to the CV backend on
  every loop. No depth sensor in this version — the backend estimates
  Z from the object's known real-world size instead (see OBJECT_SIZE_CM
  in server.py). A real depth sensor (VL53L0X, etc.) can be added back
  later via a relay MCU without any changes here or in server.py — just
  start sending a real depth_cm > 0 and it will be trusted automatically.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ---- fill these in ----
const char* WIFI_SSID   = "xxxx";
const char* WIFI_PASS   = "xxxx";
const char* BACKEND_URL = "https://bxxxx/ingest";
// ------------------------

// Common AI-Thinker-style camera pin map — verify against YOUR specific
// N16R8 dev board's schematic/silkscreen before flashing.
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5
#define Y9_GPIO_NUM        16
#define Y8_GPIO_NUM        17
#define Y7_GPIO_NUM        18
#define Y6_GPIO_NUM        12
#define Y5_GPIO_NUM        10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM        11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM      13

void setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;

  // QVGA keeps upload latency low over WiFi — that matters more than
  // image quality for a moving "follow" target.
  config.frame_size   = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;              // lower = better quality, bigger file
  config.fb_count      = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  setup_camera();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected: " + WiFi.localIP().toString());
}

void send_frame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  HTTPClient http;
  http.begin(BACKEND_URL);

  String boundary = "----ESP32Boundary";
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

  // No depth_cm field at all — server.py defaults it to -1 and falls
  // back to the camera-only size estimate automatically.
  String head = "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"frame\"; filename=\"frame.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n";
  String tail = "\r\n--" + boundary + "--\r\n";

  size_t total_len = head.length() + fb->len + tail.length();
  uint8_t* body = (uint8_t*)malloc(total_len);
  if (!body) {
    Serial.println("malloc failed for request body");
    esp_camera_fb_return(fb);
    http.end();
    return;
  }
  memcpy(body, head.c_str(), head.length());
  memcpy(body + head.length(), fb->buf, fb->len);
  memcpy(body + head.length() + fb->len, tail.c_str(), tail.length());

  int status = http.POST(body, total_len);
  if (status > 0) {
    Serial.printf("POST -> %d\n", status);
  } else {
    Serial.printf("POST failed: %s\n", http.errorToString(status).c_str());
  }

  free(body);
  http.end();
  esp_camera_fb_return(fb);
}

void loop() {
  send_frame();
  delay(100);  // ~10 fps upload — raise the delay if Render/WiFi can't keep up
}
