/*
  esp32_cam_ws_client.ino
  ESP32-S3 N16R8 + OV2640 -> streams JPEG frames to the backend over ONE
  persistent WebSocket. Replaces esp32_cam_client.ino (HTTP POST per
  frame), which paid a full HTTPS handshake + round trip on every frame
  and was capped at a few fps no matter how fast the server was.

  Why this is faster:
    - connect once, then just push frames — no per-frame handshake
    - no multipart body building / extra memory copy (raw JPEG bytes)
    - no delay() between frames
    - WiFi modem sleep disabled (a common source of latency spikes)
    - camera set to always hand over the LATEST frame, not a stale one

  Required library (Arduino IDE -> Tools -> Manage Libraries):
    "ArduinoWebsockets" by Gil Maimon

  Prints measured frames-per-second to Serial Monitor every 2 seconds
  so you can see the real number instead of guessing.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

// ---- fill these in ----
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
// wss:// (secure) for Render. For a server on your own PC use ws://<pc-ip>:8000/ws
const char* WS_URL    = "wss://bluephilic.onrender.com/ws";
// ------------------------

// Common AI-Thinker-style camera pin map — same as the other sketches.
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

WebsocketsClient wsClient;
unsigned long lastReconnectAttempt = 0;
unsigned long frameCount = 0;
unsigned long lastFpsReport = 0;

// The server replies to every frame with a small JSON result. Ignored
// for now — later this is where the car/arm controller would read
// x_cm / y_cm / z_cm. poll() must still run so replies are drained.
void onWsMessage(WebsocketsMessage message) {}

void onWsEvent(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("WebSocket connected");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("WebSocket closed");
  }
}

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

  config.frame_size   = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;              // higher number = smaller/faster, blurrier
  config.fb_count     = 2;
  // Always return the newest frame instead of an older buffered one —
  // otherwise what you see can be several frames behind reality.
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  setup_camera();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // disable modem sleep — big latency win
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected: " + WiFi.localIP().toString());

  wsClient.setInsecure();  // skip TLS cert validation (fine for a hobby project)
  wsClient.onMessage(onWsMessage);
  wsClient.onEvent(onWsEvent);
}

void loop() {
  // (Re)connect if needed — Render can drop idle/cold connections.
  if (!wsClient.available()) {
    if (millis() - lastReconnectAttempt > 2000) {
      lastReconnectAttempt = millis();
      Serial.println("Connecting WebSocket...");
      wsClient.connect(WS_URL);
    }
    delay(10);
    return;
  }

  wsClient.poll();  // drain the server's replies / keep the connection alive

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  bool ok = wsClient.sendBinary((const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("Send failed — reconnecting");
    wsClient.close();
    return;
  }

  frameCount++;
  unsigned long now = millis();
  if (now - lastFpsReport >= 2000) {
    Serial.printf("~%.1f fps\n", frameCount * 1000.0 / (now - lastFpsReport));
    frameCount = 0;
    lastFpsReport = now;
  }
}
