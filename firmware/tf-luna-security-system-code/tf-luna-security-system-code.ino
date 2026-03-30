#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TFLI2C.h>
#include "esp_camera.h"

#include "board_config.h"

// ---- Wi-Fi Station settings ----
const char *WIFI_SSID = "MOVISTAR-WIFI6-B410";
const char *WIFI_PASS = "LE2xPxeBy7jPvEKYT6Qw";

// ---- TF-Luna I2C ----
TFLI2C tflI2C;
uint8_t tflunaAddr = 0x10;

// ---- Detection settings ----
#define BASELINE_SAMPLES    20
#define BASELINE_DELAY_MS   100
#define TRIGGER_THRESHOLD   15      // cm difference to trigger capture
#define COOLDOWN_MS         10000   // 10 s cooldown after trigger
#define POLL_INTERVAL_MS    100     // sensor poll rate
#define MAX_VALID_DIST      800     // TF-Luna max range cm
#define BAD_READ_LIMIT      10      // consecutive bad reads before auto-reset

// ---- System state ----
enum SystemState { STATE_CALIBRATING, STATE_IDLE, STATE_TRIGGERED, STATE_COOLDOWN };
volatile SystemState systemState = STATE_CALIBRATING;

int baselineDistance = 0;
int lastDistance = -1;
unsigned long cooldownStart = 0;
int badReadCount = 0;
int totalResets = 0;
int totalDetections = 0;
unsigned long lastDetectionTime = 0;
unsigned long bootTime = 0;

// ---- Event log ----
#define MAX_LOG_ENTRIES 20
struct LogEntry {
  unsigned long timestamp;
  int distance;
  int baseline;
  bool photoOk;
};
LogEntry eventLog[MAX_LOG_ENTRIES];
int logCount = 0;

// ---- Captured image buffer ----
uint8_t *capturedImage = NULL;
size_t   capturedLen   = 0;

// ---- Web server ----
WebServer server(80);

// ======================= I2C =======================

void startI2C() {
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  Wire.begin(8, 9);
  Wire.setClock(100000);
}

void stopI2C() {
  Wire.end();
}

void resetTFLuna() {
  Serial.println("Resetting TF-Luna...");
  tflI2C.Hard_Reset(tflunaAddr);
  delay(3000);
  stopI2C();
  delay(50);
  startI2C();
  delay(500);
  badReadCount = 0;
  totalResets++;
  Serial.println("TF-Luna reset complete");
}

// ======================= TF-Luna =======================

// Returns raw distance (any positive value), or -1 on I2C failure
int tflunaReadDistance() {
  int16_t dist = 0;
  int16_t flux = 0;
  int16_t temp = 0;

  if (tflI2C.getData(dist, flux, temp, tflunaAddr)) {
    badReadCount = 0;
    if (dist > 0) return dist;
  }

  badReadCount++;

  // Auto-reset if too many consecutive bad reads
  if (badReadCount >= BAD_READ_LIMIT) {
    Serial.printf("Auto-reset: %d consecutive bad reads\n", badReadCount);
    resetTFLuna();
    delay(2000);
  }

  return -1;
}

// ======================= Camera =========================

static bool startCamera() {
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

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

static void stopCamera() {
  esp_camera_deinit();
}

static bool capturePhoto() {
  stopI2C();
  delay(10);

  if (!startCamera()) {
    Serial.println("Camera init failed during capture");
    startI2C();
    resetTFLuna();
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    stopCamera();
    startI2C();
    resetTFLuna();
    return false;
  }

  if (capturedImage) {
    free(capturedImage);
    capturedImage = NULL;
    capturedLen = 0;
  }

  capturedImage = (uint8_t *)malloc(fb->len);
  if (capturedImage) {
    memcpy(capturedImage, fb->buf, fb->len);
    capturedLen = fb->len;
  }
  esp_camera_fb_return(fb);

  Serial.printf("Photo captured (%u bytes)\n", capturedLen);

  stopCamera();
  delay(10);
  startI2C();
  resetTFLuna();

  return capturedImage != NULL;
}

// ======================= Event log ====================

void addLogEntry(int distance, int baseline, bool photoOk) {
  if (logCount < MAX_LOG_ENTRIES) {
    eventLog[logCount].timestamp = millis() - bootTime;
    eventLog[logCount].distance = distance;
    eventLog[logCount].baseline = baseline;
    eventLog[logCount].photoOk = photoOk;
    logCount++;
  } else {
    for (int i = 0; i < MAX_LOG_ENTRIES - 1; i++) {
      eventLog[i] = eventLog[i + 1];
    }
    eventLog[MAX_LOG_ENTRIES - 1].timestamp = millis() - bootTime;
    eventLog[MAX_LOG_ENTRIES - 1].distance = distance;
    eventLog[MAX_LOG_ENTRIES - 1].baseline = baseline;
    eventLog[MAX_LOG_ENTRIES - 1].photoOk = photoOk;
  }
}

String formatTime(unsigned long ms) {
  unsigned long secs = ms / 1000;
  unsigned long mins = secs / 60;
  unsigned long hrs = mins / 60;
  secs %= 60;
  mins %= 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hrs, mins, secs);
  return String(buf);
}

// ======================= Baseline calibration ===========

bool calibrateBaseline() {
  Serial.println("Calibrating baseline...");
  long sum = 0;
  int valid = 0;

  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    int16_t dist = 0, flux = 0, temp = 0;
    if (tflI2C.getData(dist, flux, temp, tflunaAddr)) {
      if (dist > 0) {
        Serial.printf("  Sample %d: %d cm (flux=%d)\n", valid + 1, dist, flux);
        sum += dist;
        valid++;
      }
    }
    delay(BASELINE_DELAY_MS);
  }

  if (valid > 0) {
    baselineDistance = sum / valid;
    Serial.printf("Baseline: %d cm (%d/%d valid samples)\n", baselineDistance, valid, BASELINE_SAMPLES);
    return true;
  }
  Serial.println("WARNING: No valid TF-Luna readings");
  return false;
}

// ======================= Web server handlers ============

void handleRoot() {
  const char *stateStr;
  const char *stateClass;
  switch (systemState) {
    case STATE_CALIBRATING: stateStr = "Calibrating..."; stateClass = "calibrating"; break;
    case STATE_IDLE:        stateStr = "Idle - Monitoring"; stateClass = "idle"; break;
    case STATE_TRIGGERED:   stateStr = "Triggered!"; stateClass = "triggered"; break;
    case STATE_COOLDOWN:    stateStr = "Cooldown"; stateClass = "cooldown"; break;
    default:                stateStr = "Unknown"; stateClass = "calibrating"; break;
  }

  String uptime = formatTime(millis() - bootTime);

  String html = "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='5'>"
    "<title>Luna Security</title>"
    "<style>"
    "body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:20px}"
    "h1{color:#0cf}h2{color:#aaa;font-size:1.1em;margin-top:20px}"
    "img{max-width:100%;border:2px solid #333;border-radius:8px;margin:10px 0}"
    ".status{font-size:1.3em;padding:10px 20px;border-radius:6px;display:inline-block;margin:10px}"
    ".idle{background:#1a3a1a;color:#4f4}"
    ".triggered{background:#4a1a1a;color:#f44}"
    ".cooldown{background:#3a3a1a;color:#ff4}"
    ".calibrating{background:#1a1a3a;color:#48f}"
    ".panel{text-align:left;background:#1a1a1a;padding:15px;border-radius:8px;margin:15px auto;max-width:500px}"
    ".panel b{color:#0cf}"
    ".panel .val{color:#4f4}"
    ".panel .bad{color:#f44}"
    ".mono{font-family:monospace;font-size:0.85em}"
    ".btn{display:inline-block;padding:10px 20px;margin:8px;border-radius:6px;text-decoration:none;font-size:1em;cursor:pointer;border:none}"
    ".btn-recal{background:#234;color:#0cf}"
    ".btn-photo{background:#322;color:#f84}"
    ".btn-reset{background:#332;color:#ff4}"
    "table{width:100%;border-collapse:collapse;font-family:monospace;font-size:0.8em}"
    "th{color:#0cf;text-align:left;padding:4px 8px;border-bottom:1px solid #333}"
    "td{padding:4px 8px;border-bottom:1px solid #222}"
    ".ok{color:#4f4}.fail{color:#f44}"
    "</style></head><body>"
    "<h1>Luna Security System</h1>"
    "<div class='status " + String(stateClass) + "'>" + String(stateStr) + "</div><br>";

  // Live sensor panel
  html += "<div class='panel mono'>"
    "<b>Uptime:</b> <span class='val'>" + uptime + "</span><br>"
    "<b>Baseline:</b> <span class='val'>" + String(baselineDistance) + " cm</span><br>"
    "<b>Last Reading:</b> <span class='" + String(lastDistance > 0 && lastDistance <= MAX_VALID_DIST ? "val" : "bad") + "'>"
      + (lastDistance > 0 ? String(lastDistance) + " cm" : "N/A") + "</span><br>"
    "<b>Threshold:</b> <span class='val'>" + String(TRIGGER_THRESHOLD) + " cm</span><br>"
    "<b>Total Detections:</b> <span class='val'>" + String(totalDetections) + "</span><br>"
    "<b>Sensor Resets:</b> <span class='" + String(totalResets > 3 ? "bad" : "val") + "'>" + String(totalResets) + "</span><br>"
    "<b>Bad Read Streak:</b> <span class='" + String(badReadCount > 0 ? "bad" : "val") + "'>" + String(badReadCount) + "/" + String(BAD_READ_LIMIT) + "</span><br>"
    "</div>";

  // Buttons
  html += "<a href='/recalibrate' class='btn btn-recal'>Recalibrate</a>"
    "<a href='/testphoto' class='btn btn-photo'>Test Photo</a>"
    "<a href='/hardreset' class='btn btn-reset'>Hard Reset Sensor</a><br>";

  // Last captured image
  if (capturedLen > 0) {
    html += "<h2>Last Captured Image (Detection #" + String(totalDetections) + ")</h2>"
      "<img src='/image' alt='Captured'><br>"
      "<p style='color:#888;font-size:0.85em'>Size: " + String(capturedLen) + " bytes";
    if (lastDetectionTime > 0) {
      html += " | Captured at " + formatTime(lastDetectionTime - bootTime);
    }
    html += "</p>";
  } else {
    html += "<p>No image captured yet.</p>";
  }

  // Event log
  if (logCount > 0) {
    html += "<h2>Detection Log</h2><div class='panel'><table>"
      "<tr><th>#</th><th>Time</th><th>Dist</th><th>Baseline</th><th>Diff</th><th>Photo</th></tr>";
    for (int i = logCount - 1; i >= 0; i--) {
      int diff = abs(eventLog[i].distance - eventLog[i].baseline);
      html += "<tr><td>" + String(i + 1) + "</td>"
        "<td>" + formatTime(eventLog[i].timestamp) + "</td>"
        "<td>" + String(eventLog[i].distance) + " cm</td>"
        "<td>" + String(eventLog[i].baseline) + " cm</td>"
        "<td>" + String(diff) + " cm</td>"
        "<td class='" + String(eventLog[i].photoOk ? "ok" : "fail") + "'>"
        + String(eventLog[i].photoOk ? "OK" : "FAIL") + "</td></tr>";
    }
    html += "</table></div>";
  }

  html += "<br><p style='color:#555;font-size:0.75em'>Auto-refreshes every 5 seconds</p>"
    "</body></html>";

  server.send(200, "text/html", html);
}

void handleImage() {
  if (capturedImage && capturedLen > 0) {
    server.send_P(200, "image/jpeg", (const char *)capturedImage, capturedLen);
  } else {
    server.send(204, "text/plain", "No image");
  }
}

void handleRecalibrate() {
  resetTFLuna();
  delay(2000);
  calibrateBaseline();
  systemState = STATE_IDLE;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleTestPhoto() {
  capturePhoto();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleHardReset() {
  resetTFLuna();
  delay(2000);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStatus() {
  String json = "{\"state\":\"";
  switch (systemState) {
    case STATE_CALIBRATING: json += "calibrating"; break;
    case STATE_IDLE:        json += "idle"; break;
    case STATE_TRIGGERED:   json += "triggered"; break;
    case STATE_COOLDOWN:    json += "cooldown"; break;
  }
  json += "\",\"baseline\":" + String(baselineDistance);
  json += ",\"lastDist\":" + String(lastDistance);
  json += ",\"detections\":" + String(totalDetections);
  json += ",\"resets\":" + String(totalResets);
  json += ",\"badReads\":" + String(badReadCount);
  json += ",\"hasImage\":" + String(capturedLen > 0 ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ======================= Setup ==========================

void setup() {
  Serial.begin(115200);
  delay(800);
  bootTime = millis();
  Serial.println("\n=== Luna Security System ===");

  // Start I2C (camera off — shares GPIO 8/9)
  startI2C();

  // Hard reset TF-Luna to factory defaults
  resetTFLuna();

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 30) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected!");
    Serial.printf("Web UI: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(" FAILED - continuing without WiFi");
  }

  // Web server routes
  server.on("/", handleRoot);
  server.on("/image", handleImage);
  server.on("/status", handleStatus);
  server.on("/recalibrate", handleRecalibrate);
  server.on("/testphoto", handleTestPhoto);
  server.on("/hardreset", handleHardReset);
  server.begin();

  // Wait for sensor to settle then calibrate
  Serial.println("Waiting 10 seconds for sensor to settle...");
  delay(10000);
  calibrateBaseline();

  systemState = STATE_IDLE;
  Serial.println("System armed - monitoring started");
}

// ======================= Main loop ======================

void loop() {
  server.handleClient();

  static unsigned long lastPoll = 0;
  unsigned long now = millis();

  if (systemState == STATE_COOLDOWN) {
    if (now - cooldownStart >= COOLDOWN_MS) {
      systemState = STATE_IDLE;
      Serial.println("Cooldown ended - resuming monitoring");
    }
    return;
  }

  if (systemState == STATE_IDLE && (now - lastPoll >= POLL_INTERVAL_MS)) {
    lastPoll = now;

    int dist = tflunaReadDistance();
    if (dist > 0) lastDistance = dist;
    if (dist < 0) return;

    int diff = abs(dist - baselineDistance);

    if (diff >= TRIGGER_THRESHOLD) {
      Serial.printf("INTRUSION #%d! dist=%d cm (baseline=%d, diff=%d)\n",
                     totalDetections + 1, dist, baselineDistance, diff);
      systemState = STATE_TRIGGERED;
      totalDetections++;
      lastDetectionTime = millis();

      bool photoOk = capturePhoto();
      addLogEntry(dist, baselineDistance, photoOk);

      systemState = STATE_COOLDOWN;
      cooldownStart = millis();
      Serial.println("Entering cooldown...");
    }
  }
}
