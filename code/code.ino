

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ==================== WIFI ====================
const char* ssid = "TT";
const char* password = "123456788";

// ==================== FASTAPI DETECT ====================
String FASTAPI_URL = "http://192.168.39.224:8000/api/detect";
String JWT_TOKEN = "test_token";

// ==================== FIREBASE REALTIME DATABASE ====================
// URL để stream dữ liệu từ Firebase Realtime Database (SSE)
const char* FIREBASE_HOST = "thef-detect-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* FIREBASE_STREAM_PATH = "/door_command.json";

// ==================== HARDWARE PINS ====================
#define PIR_SENSOR_PIN 2
#define LED_RED_PIN 14
#define BUZZER_PIN 1
#define SERVO_PIN 3

// ==================== CAMERA PINS ====================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      15
#define SIOD_GPIO_NUM       4
#define SIOC_GPIO_NUM       5
#define Y9_GPIO_NUM        16
#define Y8_GPIO_NUM        17
#define Y7_GPIO_NUM        18
#define Y6_GPIO_NUM        12
#define Y5_GPIO_NUM        10
#define Y4_GPIO_NUM         8
#define Y3_GPIO_NUM         9
#define Y2_GPIO_NUM        11
#define VSYNC_GPIO_NUM      6
#define HREF_GPIO_NUM       7
#define PCLK_GPIO_NUM      13

// ==================== CONFIG ====================
const unsigned long MOTION_TIMEOUT = 10000;
const int MAX_FRAMES = 3;
const unsigned long UPLOAD_DELAY = 1000;
int frameIndex = 0;
bool alarmActive = false;
bool personPresent = false;
bool alarmSuppressed = false;
unsigned long lastMotionTime = 0;
unsigned long lastUploadTime = 0;
unsigned long lastDebugLogTime = 0;
const unsigned long DEBUG_LOG_INTERVAL = 3000;  // In log debug mỗi 3 giây
bool pirWasLow = true;  // Yêu cầu PIR phải về LOW trước khi phát hiện chuyển động mới

// ==================== FIREBASE STREAM ====================
WiFiClientSecure streamClient;
String lastCommandId = "";  // Lưu command_id đã xử lý để tránh xử lý trùng
bool streamConnected = false;
unsigned long lastStreamReconnect = 0;
const unsigned long STREAM_RECONNECT_INTERVAL = 5000;

// ==================== SERVO ====================
Servo doorServo;
const int SERVO_CLOSED = 0;
const int SERVO_OPEN = 90;

// =========================================================
//                CAMERA INITIALIZATION
// =========================================================
void setupCamera() {
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

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 20;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    ESP.restart();
  }

  Serial.println("📸 Camera ready");
}

// =========================================================
//            SEND FRAME TO FASTAPI /detect
// =========================================================
String uploadToFastAPIDetect(camera_fb_t *fb) {
  HTTPClient http;

  http.begin(FASTAPI_URL);
  http.addHeader("Authorization", "Bearer " + JWT_TOKEN);

  String boundary = "----ESP32Boundary782346";
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

  String bodyStart =
      "--" + boundary + "\r\n"
      "Content-Disposition: form-data; name=\"file\"; filename=\"frame.jpg\"\r\n"
      "Content-Type: image/jpeg\r\n\r\n";

  String bodyEnd = "\r\n--" + boundary + "--\r\n";

  size_t totalLen = bodyStart.length() + fb->len + bodyEnd.length();
  
  uint8_t *body = (uint8_t *)malloc(totalLen);
  if (!body) {
    Serial.println("❌ Failed to allocate memory for body");
    return "";
  }

  size_t offset = 0;
  memcpy(body + offset, bodyStart.c_str(), bodyStart.length());
  offset += bodyStart.length();
  memcpy(body + offset, fb->buf, fb->len);
  offset += fb->len;
  memcpy(body + offset, bodyEnd.c_str(), bodyEnd.length());

  int httpCode = http.POST(body, totalLen);
  free(body);

  if (httpCode <= 0) {
    Serial.printf("❌ HTTP POST failed: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return "";
  }

  Serial.printf("✅ HTTP Response code: %d\n", httpCode);
  String response = http.getString();
  http.end();
  return response;
}

// =========================================================
//               ALARM CONTROL
// =========================================================
void startAlarm() {
  if (!alarmActive) {
    alarmActive = true;
    Serial.println("🚨 ALARM ON");
  }
}

void stopAlarm() {
  if (alarmActive) {
    alarmActive = false;
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("🟢 ALARM OFF");
  }
}

void runAlarm() {
  if (!alarmActive) return;
  static unsigned long lastBlink = 0;
  static bool state = false;

  if (millis() - lastBlink > 250) {
    lastBlink = millis();
    state = !state;
    digitalWrite(LED_RED_PIN, state);
    digitalWrite(BUZZER_PIN, state);
  }
}

// =========================================================
//               DOOR CONTROL
// =========================================================
void openDoor() {
  Serial.println("🚪 Opening door...");
  doorServo.write(SERVO_OPEN);
  delay(4000);
  Serial.println("🚪 Closing door...");
  doorServo.write(SERVO_CLOSED);
}

// =========================================================
//           PROCESS JSON RESPONSE FROM FASTAPI
// =========================================================
void processDetectResult(String apiResponse) {
  if (apiResponse.length() == 0) return;

  Serial.println("📩 Detect API Response:");
  Serial.println(apiResponse);

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, apiResponse);

  if (err) {
    Serial.println("❌ JSON parse error");
    return;
  }

  bool alert = doc["alert"] | false;
  bool openSig = doc["open"] | false;

  if (alert) {
    if (!alarmActive && !alarmSuppressed) {
      Serial.println("🚨 ALERT → START ALARM");
      startAlarm();
    }
  } else {
    if (alarmActive) {
      Serial.println("ℹ️ NO ALERT → STOP ALARM");
      stopAlarm();
    }
  }

  if (openSig) {
    Serial.println("🚪 OPEN DOOR COMMAND");
    stopAlarm();
    openDoor();
    alarmSuppressed = true;
  }
}

// =========================================================
//                    MOTION DETECT
// =========================================================
bool checkMotion() {
  bool motion = digitalRead(PIR_SENSOR_PIN);

  // Nếu không có chuyển động, đánh dấu PIR đã về LOW
  if (!motion) {
    pirWasLow = true;
  }

  // Chỉ phát hiện chuyển động MỚI khi:
  // 1. PIR đang HIGH (có chuyển động)
  // 2. pirWasLow = true (PIR đã từng về LOW trước đó)
  // 3. personPresent = false (chưa có người)
  if (motion && pirWasLow) {
    lastMotionTime = millis();
    if (!personPresent) {
      personPresent = true;
      pirWasLow = false;  // Reset - phải đợi PIR về LOW lần nữa
      Serial.println("👤 Motion detected!");
      return true;
    }
  }

  // Cập nhật lastMotionTime nếu vẫn có motion (để reset timeout)
  if (motion && personPresent) {
    lastMotionTime = millis();
  }

  if (personPresent && (millis() - lastMotionTime > MOTION_TIMEOUT)) {
    personPresent = false;
    Serial.println("👋 Person left (timeout)");
    
    // Tự động tắt alarm khi hết chuyển động
    stopAlarm();
    alarmSuppressed = false;  // Reset để lần sau có thể báo động lại
    // pirWasLow vẫn giữ nguyên - phải đợi PIR về LOW trước khi detect lại
  }

  return false;
}

// =========================================================
//                    FRAME UPLOAD
// =========================================================
void uploadFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    return;
  }

  String response = uploadToFastAPIDetect(fb);
  esp_camera_fb_return(fb);

  processDetectResult(response);
}

// =========================================================
//       FIREBASE REALTIME DATABASE - SSE STREAM
// =========================================================
void connectToFirebaseStream() {
  Serial.println("🔥 Connecting to Firebase Realtime Database stream...");
  
  streamClient.setInsecure();  // Bỏ qua SSL certificate check (cho đơn giản)
  
  if (!streamClient.connect(FIREBASE_HOST, 443)) {
    Serial.println("❌ Failed to connect to Firebase");
    streamConnected = false;
    return;
  }
  
  // Gửi HTTP GET request với Accept: text/event-stream để nhận SSE
  String request = String("GET ") + FIREBASE_STREAM_PATH + " HTTP/1.1\r\n" +
                   "Host: " + FIREBASE_HOST + "\r\n" +
                   "Accept: text/event-stream\r\n" +
                   "Connection: keep-alive\r\n\r\n";
  
  streamClient.print(request);
  
  // Đọc và bỏ qua HTTP headers
  while (streamClient.connected()) {
    String line = streamClient.readStringUntil('\n');
    if (line == "\r") {
      break;  // Headers đã kết thúc
    }
  }
  
  streamConnected = true;
  Serial.println("✅ Firebase stream connected! Listening for door commands...");
}

void processFirebaseEvent(String eventData) {
  // Firebase SSE gửi dữ liệu dạng:
  // event: put
  // data: {"path":"/","data":{...}}
  
  Serial.println("📨 Firebase event received:");
  Serial.println(eventData);
  
  // Parse JSON data
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, eventData);
  
  if (err) {
    Serial.println("⚠️ JSON parse error for Firebase event");
    return;
  }
  
  // Lấy data từ event
  JsonObject data = doc["data"];
  if (data.isNull()) {
    // Có thể là data trực tiếp (không có wrapper)
    const char* action = doc["action"];
    const char* commandId = doc["command_id"];
    bool executed = doc["executed"] | true;
    
    if (action && commandId && strcmp(action, "open") == 0 && !executed) {
      if (String(commandId) != lastCommandId) {
        Serial.println("🚪🔔 NEW DOOR OPEN COMMAND RECEIVED!");
        lastCommandId = String(commandId);
        
        // Mở cửa
        stopAlarm();
        openDoor();
        alarmSuppressed = true;
      }
    }
    return;
  }
  
  // Trường hợp data có wrapper
  const char* action = data["action"];
  const char* commandId = data["command_id"];
  bool executed = data["executed"] | true;
  
  if (action && commandId && strcmp(action, "open") == 0 && !executed) {
    if (String(commandId) != lastCommandId) {
      Serial.println("🚪🔔 NEW DOOR OPEN COMMAND RECEIVED!");
      lastCommandId = String(commandId);
      
      // Mở cửa
      stopAlarm();
      openDoor();
      alarmSuppressed = true;
    }
  }
}

void handleFirebaseStream() {
  if (!streamConnected) {
    // Thử kết nối lại
    if (millis() - lastStreamReconnect >= STREAM_RECONNECT_INTERVAL) {
      lastStreamReconnect = millis();
      connectToFirebaseStream();
    }
    return;
  }
  
  // Kiểm tra kết nối còn sống không
  if (!streamClient.connected()) {
    Serial.println("⚠️ Firebase stream disconnected");
    streamConnected = false;
    return;
  }
  
  // Đọc dữ liệu SSE nếu có
  while (streamClient.available()) {
    String line = streamClient.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("data:")) {
      String eventData = line.substring(5);  // Bỏ "data:" prefix
      eventData.trim();
      
      if (eventData.length() > 0 && eventData != "null") {
        processFirebaseEvent(eventData);
      }
    }
  }
}

// =========================================================
//                          SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  doorServo.attach(SERVO_PIN);
  doorServo.write(SERVO_CLOSED);

  // WIFI
  Serial.println("📡 Connecting WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected");
  Serial.println(WiFi.localIP());

  setupCamera();
  
  // Kết nối Firebase Realtime Database stream
  connectToFirebaseStream();
}

// =========================================================
//                          LOOP
// =========================================================
void loop() {
  // Lắng nghe Firebase stream cho lệnh mở cửa (real-time!)
  handleFirebaseStream();

  if (checkMotion()) {
    frameIndex = 0;
    alarmSuppressed = false;
    lastUploadTime = millis() - UPLOAD_DELAY;
  }

  if (personPresent && frameIndex < MAX_FRAMES) {
    if (millis() - lastUploadTime >= UPLOAD_DELAY) {
      uploadFrame();
      frameIndex++;
      lastUploadTime = millis();
      Serial.printf("📸 Sent frame %d/%d\n", frameIndex, MAX_FRAMES);
    }
  }

  // Debug log - hiển thị trạng thái hệ thống mỗi 3 giây
  if (millis() - lastDebugLogTime >= DEBUG_LOG_INTERVAL) {
    lastDebugLogTime = millis();
    if (personPresent) {
      unsigned long timeRemaining = MOTION_TIMEOUT - (millis() - lastMotionTime);
      Serial.printf("⏳ Status: personPresent=%s, frameIndex=%d/%d, alarm=%s, timeout in %.1fs\n",
                    personPresent ? "true" : "false",
                    frameIndex, MAX_FRAMES,
                    alarmActive ? "ON" : "OFF",
                    timeRemaining / 1000.0);
    }
  }

  runAlarm();
  delay(50);
}
