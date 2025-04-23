#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "motor_control.h"
#include "bluetooth_control.h"
#include "esp_bt.h"
#include "line_algorithm.h"
#include <WiFi.h>

#define ENABLE_STREAMING 1  // Set to enable camera HTTP stream


// Replace with your network credentials
const char* ssid = "hampter";
const char* password = "rarepepe";

// Select camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "camera_index.h"


void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_96X96;
  config.jpeg_quality = 12;
  config.fb_count = 3;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  

  
  // Init camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  // Default to 96x96
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_96X96);


  //esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  #if ENABLE_STREAMING
  // Initialize Wi-Fi first
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.println(WiFi.localIP());

  WiFi.setSleep(true); // Necessary when using both bluetooth and wifi
  #endif

  // Now Bluetooth
 // bluetooth_setup();

  // Motors
  motor_setup();

  // Camera
#if ENABLE_STREAMING
  startCameraServer();
#endif


}

void loop() {
  // Controller code disabled for now
  // static uint32_t last_bt_update = 0;
  // static byte_t lastDirection = Stop;
  // uint32_t now = millis();

  // if (now - last_bt_update >= 50) {
  //   lastDirection = bluetooth_update();
  //   last_bt_update = now;
  // }

  static float lastValidAngle = NAN;
  static uint32_t lastLineProcess = 0;
  static int droppedFrames = 0;

  const uint32_t LINE_PROCESS_INTERVAL_MS = 200; // ~5 FPS
  uint32_t now = millis();

  // Process a frame every 200 ms
  if (now - lastLineProcess >= LINE_PROCESS_INTERVAL_MS) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      lastValidAngle = get_line_angle_from_frame(fb);
      esp_camera_fb_return(fb);
    } else {
      droppedFrames++;
      if (droppedFrames % 10 == 0) {
        Serial.printf("[Warning] Dropped frames: %d\n", droppedFrames);
      }

      // Delay to let camera recover
      delay(10);
    }

    lastLineProcess = now;
  }

  // Always run motor logic with the most recent valid angle
  controlMotors(MotorsOff, lastValidAngle);

  delay(1); // Yield to background tasks
}
