/*
 * code_camv2.ino  -  ESP32-CAM (AI Thinker) stream MJPEG cho robot tránh vật cản
 *
 * Thay đổi so với v1:
 *  - Khóa cân bằng trắng / phơi sáng / gain để màu HSV ổn định (LOCK_CAMERA)
 *  - Độ phân giải chọn bằng 1 dòng #define (mặc định QVGA 320x240).
 *    Python v5 tự xử lý theo kích thước khung thực nhận được nên đổi thoải mái.
 *  - Dùng PSRAM nếu có, thêm kiểm tra timeout WiFi (tự restart)
 *
 * Nạp code: nối IO0 xuống GND -> reset -> upload -> RÚT IO0 ra -> reset.
 * Cấp nguồn 5V riêng (>= 500mA, tốt nhất 1-2A), không lấy chung với motor.
 */
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ===== WIFI CONFIG (WiFi 2.4GHz, cùng mạng với laptop và ESP8266) =====
const char* ssid     = "IQOO13";
const char* password = "11111111";

// ===== TÙY CHỌN =====
// FRAMESIZE_QVGA  = 320x240 (chi tiết hơn)
// FRAMESIZE_QQVGA = 160x120 (nhẹ nhất, dùng nếu stream bị lag)
#define CAM_FRAME_SIZE   FRAMESIZE_QVGA
#define CAM_JPEG_QUALITY 15      // 0-63, số càng lớn ảnh càng nén (nhẹ), 10-20 là hợp lý

// 1 = khóa tự động chỉnh sáng/màu (màu ổn định cho HSV). 0 = để camera tự động.
#define LOCK_CAMERA      1
#define AEC_VALUE        500     // độ phơi sáng 0-1200: tăng nếu ảnh tối, giảm nếu cháy sáng
#define AGC_GAIN         5       // 0-30

// ===== CAMERA PINS (AI Thinker) =====
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

// ===== STREAM SERVER =====
httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      size_t hlen = snprintf(part_buf, sizeof(part_buf),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        (unsigned int)fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, "\r\n", 2);
      }
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break;   // client ngắt kết nối -> thoát, chờ lần kết nối sau
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = CAM_FRAME_SIZE;
  config.jpeg_quality = CAM_JPEG_QUALITY;
  config.fb_count     = 2;                       // 1 khung đang gửi, 1 khung đang chụp
  config.grab_mode    = CAMERA_GRAB_LATEST;      // luôn lấy khung mới nhất -> giảm trễ
  config.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

#if LOCK_CAMERA
  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);          // tắt cân bằng trắng tự động
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);     // tắt phơi sáng tự động
  s->set_aec_value(s, AEC_VALUE);
  s->set_gain_ctrl(s, 0);         // tắt gain tự động
  s->set_agc_gain(s, AGC_GAIN);
#endif

  WiFi.mode(WIFI_STA);
  // ESP8266 dùng .201, ESP32-CAM dùng .200 (đổi đuôi cho mỗi board)
  IPAddress local_IP(10, 64, 131, 200);
  IPAddress gateway(10, 64, 131, 14);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("Connecting WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 30000) {   // 30s không vào được WiFi -> khởi động lại thử lại
      Serial.println("\nWiFi timeout, restarting...");
      ESP.restart();
    }
  }

  Serial.println("\nWiFi connected");
  Serial.print("Camera Stream: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");   // <--- điền IP này vào CAM_URL trong algo_v5.py

  startCameraServer();
}

void loop() {
  // Nếu rớt WiFi giữa chừng thì khởi động lại để tự nối lại
  if (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    if (WiFi.status() != WL_CONNECTED) ESP.restart();
  }
  delay(1000);
}
