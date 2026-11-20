/*
 * dk_dong_co_wifi_v3.ino  -  ESP8266 điều khiển 2 động cơ qua WiFi (L298N)
 *
 * Thay đổi so với v2:
 *  - analogWriteRange(255): đúng thang 0-255 như comment (mặc định ESP8266 là 0-1023)
 *  - Watchdog: tự dừng motor nếu > CMD_TIMEOUT ms không nhận lệnh (mất WiFi / mất laptop)
 *  - Thêm /drive?l=..&r=.. : điều khiển 2 bánh độc lập (-255..255) để lái mượt
 *  - Tắt WiFi sleep để giảm trễ, tự kết nối lại khi mất WiFi
 *  - Giữ nguyên /forward /back /left /right /stop để test tay
 *
 * Đấu nối (giữ nguyên như v2):
 *   D1->IN1  D2->IN2  D5->ENA   (bánh trái)
 *   D8->IN3  D7->IN4  D6->ENB   (bánh phải)   * tháo jumper ENA/ENB trên L298N
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ========== WIFI CONFIG (điền WiFi 2.4GHz của bạn) ==========
const char* ssid     = "IQOO13";
const char* password = "11111111";

// ========== PIN DEFINE ==========
#define M1_IN1 D1   // Bánh trái
#define M1_IN2 D2
#define M1_EN  D5   // PWM

#define M2_IN1 D8   // Bánh phải
#define M2_IN2 D7
#define M2_EN  D6   // PWM

// ========== TỐC ĐỘ (0 - 255), dùng cho /forward /back /left /right ==========
// Nếu xe chỉ "ù" không chạy -> tăng lên. Chạy quá nhanh -> giảm xuống.
#define BASE_SPEED   100
#define TURN_SPEED   110

// Tự dừng nếu quá thời gian này (ms) không nhận lệnh nào.
// Python gửi lại lệnh mỗi 0.5s nên 2000ms là dư an toàn.
#define CMD_TIMEOUT  2000

ESP8266WebServer server(80);

unsigned long lastCmd = 0;
bool motorsRunning = false;

// ========== MOTOR CONTROL ==========
void stopMotor() {
  analogWrite(M1_EN, 0);
  analogWrite(M2_EN, 0);
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);
  motorsRunning = false;
}

// speed: -255..255. Dương = tiến, âm = lùi
void setMotor(int speedL, int speedR) {
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);

  // ĐẢO ÁNH XẠ: M1 thực tế là bánh PHẢI
  //             M2 thực tế là bánh TRÁI

  // --- M1: BÁNH PHẢI ---
  if (speedR >= 0) {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    analogWrite(M1_EN, speedR);
  } else {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    analogWrite(M1_EN, -speedR);
  }

  // --- M2: BÁNH TRÁI ---
  if (speedL >= 0) {
    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
    analogWrite(M2_EN, speedL);
  } else {
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
    analogWrite(M2_EN, -speedL);
  }

  motorsRunning = (speedL != 0 || speedR != 0);
}


// Trả lời OK và ghi nhận thời điểm nhận lệnh (để watchdog biết còn "sống")
void reply() {
  lastCmd = millis();
  server.send(200, "text/plain", "OK");
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT); pinMode(M1_EN, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT); pinMode(M2_EN, OUTPUT);

  analogWriteRange(255);   // QUAN TRỌNG: thang PWM 0-255
  stopMotor();

  // KẾT NỐI WIFI
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);   // giảm trễ
  // ESP8266 dùng .201, ESP32-CAM dùng .200 (đổi đuôi cho mỗi board)
  IPAddress local_IP(10, 64, 131, 201);
  IPAddress gateway(10, 64, 131, 14);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());   // <--- điền IP này vào ROBOT_IP trong algo_v5.py

  // ----- CÁC LỆNH ĐIỀU KHIỂN -----
  // "/" chỉ để kiểm tra kết nối, KHÔNG tính là lệnh điều khiển
  server.on("/", []() { server.send(200, "text/plain", "ESP8266 Robot Ready"); });

  server.on("/forward", []() { setMotor(BASE_SPEED, BASE_SPEED);   reply(); });
  server.on("/back",    []() { setMotor(-BASE_SPEED, -BASE_SPEED); reply(); });
  server.on("/left",    []() { setMotor(-TURN_SPEED, TURN_SPEED);  reply(); });  // xoay trái tại chỗ
  server.on("/right",   []() { setMotor(TURN_SPEED, -TURN_SPEED);  reply(); });  // xoay phải tại chỗ
  server.on("/stop",    []() { stopMotor();                        reply(); });

  // Điều khiển 2 bánh độc lập: /drive?l=100&r=60  (-255..255)
  server.on("/drive", []() {
    if (!server.hasArg("l") || !server.hasArg("r")) {
      server.send(400, "text/plain", "need l and r");
      return;
    }
    setMotor(server.arg("l").toInt(), server.arg("r").toInt());
    reply();
  });

  server.begin();
}

// ========== LOOP ==========
void loop() {
  server.handleClient();

  // Watchdog: mất lệnh quá lâu hoặc mất WiFi -> dừng
  if (motorsRunning) {
    if (millis() - lastCmd > CMD_TIMEOUT || WiFi.status() != WL_CONNECTED) {
      stopMotor();
    }
  }
}
