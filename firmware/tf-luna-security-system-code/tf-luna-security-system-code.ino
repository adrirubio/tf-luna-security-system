// ============================================================================
//  Security System — TF-Luna + ESP32-S3 Camera
// ----------------------------------------------------------------------------
//  IMPORTANT: In Arduino IDE select a partition scheme with at least ~2 MB of
//  LittleFS/SPIFFS space (e.g. "8M with spiffs (3MB APP/1.5MB SPIFFS)" or
//  larger). LittleFS reuses the SPIFFS partition. Without enough space the
//  circular photo buffer will drop captures.
// ============================================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <TFLI2C.h>
#include <LittleFS.h>
#include <time.h>
#include "esp_camera.h"

// mDNS hostname: http://security-system.local/
#define MDNS_HOSTNAME "security-system"

#include "board_config.h"

// ---- Wi-Fi Station settings ----
const char *WIFI_SSID = "MOVISTAR-WIFI6-B410";
const char *WIFI_PASS = "LE2xPxeBy7jPvEKYT6Qw";

// ---- TF-Luna I2C ----
TFLI2C tflI2C;
uint8_t tflunaAddr = 0x10;

// ---- Detection settings ----
#define WARMUP_MS           5000     // wait 5s before touching the sensor
#define BASELINE_MS         10000    // sample baseline for 10s
#define BASELINE_INTERVAL   100      // ms between baseline samples
#define TRIGGER_THRESHOLD   20       // cm — clearly something in the way
#define COOLDOWN_MS         250      // ms lockout after a capture (fast burst)
#define POLL_INTERVAL_MS    100      // monitoring sample interval
#define MAX_VALID_DIST      800      // TF-Luna max range cm

// ---- Sensor health ----
#define BAD_READ_LIMIT      8        // consecutive bad reads → degraded mode
#define RESET_RECOVER_MS    1500     // time to allow after a quick reset

// ---- Photo storage ----
#define MAX_PHOTOS          100
#define META_PATH           "/meta.bin"

// ---- System state ----
enum SystemState { STATE_WARMUP, STATE_CALIBRATING, STATE_MONITORING, STATE_TRIGGERED, STATE_COOLDOWN };
volatile SystemState systemState = STATE_WARMUP;

int  baselineDistance  = 0;
int  lastDistance      = -1;
int  lastBreakDistance = 0;
unsigned long lastBreakEpoch = 0;

unsigned long stateStart    = 0;
unsigned long cooldownStart = 0;
unsigned long bootMillis    = 0;

long calibSum   = 0;
int  calibCount = 0;

// ---- Sensor health tracking ----
int  badReadStreak   = 0;
bool degraded        = false;
int  totalQuickResets = 0;

// ---- Persistent metadata (saved to LittleFS) ----
struct __attribute__((packed)) Meta {
  uint32_t magic;                  // 0x5345434B "SECK"
  uint32_t nextIndex;              // next slot to write (0..MAX_PHOTOS-1)
  uint32_t totalCaptures;          // lifetime counter
  uint32_t epoch[MAX_PHOTOS];      // unix epoch for each slot (0 if empty)
  int16_t  distance[MAX_PHOTOS];   // distance at trigger
  int16_t  baseline[MAX_PHOTOS];   // baseline at trigger
};
Meta meta;

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

// ======================= TF-Luna =======================

// Aggressive boot-time recovery: scan the bus, hard-reset whatever
// looks like a TF-Luna at any address, and wait for it to come back
// at the default 0x10. Survives a sensor whose EEPROM got corrupted
// into a non-default I2C address.
void tflunaForceFactoryReset() {
  Serial.println("TF-Luna: forcing factory reset...");

  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("  Attempt %d/3\n", attempt);

    // 1. Scan bus
    uint8_t found[8];
    int nFound = 0;
    for (uint8_t a = 0x08; a <= 0x77 && nFound < 8; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        Serial.printf("    bus device @ 0x%02X\n", a);
        found[nFound++] = a;
      }
    }
    if (nFound == 0) {
      Serial.println("    no I2C devices on bus — check wiring/power");
      delay(500);
      continue;
    }

    // 2. Hard-reset every responding address (TF-Luna ignores reset
    //    sent to wrong address — safe to spray)
    for (int i = 0; i < nFound; i++) {
      Serial.printf("    Hard_Reset(0x%02X)\n", found[i]);
      tflI2C.Hard_Reset(found[i]);
      delay(200);
    }

    // 3. Wait for sensor to reboot
    Serial.println("    waiting 4s for reboot...");
    delay(4000);

    // 4. Bounce the bus so the controller forgets any stuck state
    stopI2C();
    delay(100);
    startI2C();
    delay(300);

    // 5. Verify default address is alive
    Wire.beginTransmission(tflunaAddr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    TF-Luna present @ 0x%02X — restoring defaults\n", tflunaAddr);
      tflI2C.Soft_Reset(tflunaAddr);    delay(800);
      tflI2C.Set_Enable(tflunaAddr);    delay(50);
      tflI2C.Set_Cont_Mode(tflunaAddr); delay(50);
      uint16_t fps = 10;               // 10Hz — match our poll rate, save power
      tflI2C.Set_Frame_Rate(fps, tflunaAddr); delay(50);
      // NOT calling Save_Settings — protect EEPROM from wear
      Serial.println("TF-Luna: factory reset complete");
      return;
    }
    Serial.println("    sensor not at 0x10 yet, retrying...");
  }
  Serial.println("TF-Luna: factory reset gave up — continuing anyway");
}

// Quick reset: bounce I2C bus + soft-reset sensor. Same effect as
// unplug/replug but without touching the cable. Takes ~1.5s.
void tflunaQuickReset() {
  Serial.println("TF-Luna: quick reset (I2C bounce + soft reset)...");
  totalQuickResets++;
  stopI2C();
  delay(100);
  startI2C();
  delay(200);
  tflI2C.Soft_Reset(tflunaAddr);
  delay(800);
  tflI2C.Set_Enable(tflunaAddr);
  delay(50);
  tflI2C.Set_Cont_Mode(tflunaAddr);
  delay(50);
  uint16_t fps = 10;                  // 10Hz — match poll rate, save power
  tflI2C.Set_Frame_Rate(fps, tflunaAddr);
  delay(50);
  badReadStreak = 0;
  Serial.println("TF-Luna: quick reset done");
}

int tflunaReadDistance() {
  int16_t dist = 0, flux = 0, temp = 0;
  bool ok = tflI2C.getData(dist, flux, temp, tflunaAddr);

  if (ok && dist > 0 && dist <= MAX_VALID_DIST && flux > 20) {
    // Good read — healthy signal
    if (badReadStreak > 0) badReadStreak--;
    if (badReadStreak == 0 && degraded) {
      degraded = false;
      Serial.println("Sensor: readings recovered");
    }
    return dist;
  }

  // Log what the sensor actually returned so we can diagnose
  if (ok) {
    Serial.printf("Sensor: suspect read dist=%d flux=%d temp=%d\n",
                  dist, flux, temp);
  }

  badReadStreak++;
  if (badReadStreak >= BAD_READ_LIMIT && !degraded) {
    degraded = true;
    Serial.printf("Sensor: DEGRADED (%d consecutive bad reads)\n", badReadStreak);
    tflunaQuickReset();
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
  config.jpeg_quality = 18;          // higher number = smaller file
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

// ======================= Storage ========================

String photoPath(uint32_t slot) {
  char buf[20];
  snprintf(buf, sizeof(buf), "/img_%03lu.jpg", (unsigned long)slot);
  return String(buf);
}

void saveMeta() {
  File f = LittleFS.open(META_PATH, "w");
  if (!f) {
    Serial.println("saveMeta: open failed");
    return;
  }
  f.write((const uint8_t *)&meta, sizeof(meta));
  f.close();
}

void loadMeta() {
  memset(&meta, 0, sizeof(meta));
  meta.magic = 0x5345434B;
  if (!LittleFS.exists(META_PATH)) {
    Serial.println("No meta file — starting fresh");
    return;
  }
  File f = LittleFS.open(META_PATH, "r");
  if (!f) return;
  if (f.size() != sizeof(Meta)) {
    Serial.println("meta.bin size mismatch — resetting");
    f.close();
    memset(&meta, 0, sizeof(meta));
    meta.magic = 0x5345434B;
    return;
  }
  f.read((uint8_t *)&meta, sizeof(meta));
  f.close();
  if (meta.magic != 0x5345434B) {
    Serial.println("meta.bin magic bad — resetting");
    memset(&meta, 0, sizeof(meta));
    meta.magic = 0x5345434B;
    return;
  }
  Serial.printf("Loaded meta: nextIndex=%u totalCaptures=%u\n",
                meta.nextIndex, meta.totalCaptures);
}

uint32_t storedPhotoCount() {
  return meta.totalCaptures < MAX_PHOTOS ? meta.totalCaptures : MAX_PHOTOS;
}

// Slot of newest photo (nth = 0 means newest)
int32_t nthNewestSlot(uint32_t n) {
  uint32_t count = storedPhotoCount();
  if (n >= count) return -1;
  // nextIndex points to where the NEXT photo will be written
  int32_t slot = ((int32_t)meta.nextIndex - 1 - (int32_t)n);
  slot %= MAX_PHOTOS;
  if (slot < 0) slot += MAX_PHOTOS;
  return slot;
}

// ======================= Capture ========================

bool capturePhoto(int distanceAtTrigger) {
  stopI2C();
  delay(10);

  if (!startCamera()) {
    Serial.println("Camera init failed");
    startI2C();
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    stopCamera();
    startI2C();
    return false;
  }

  uint32_t slot = meta.nextIndex % MAX_PHOTOS;
  String path = photoPath(slot);
  if (LittleFS.exists(path)) LittleFS.remove(path);

  bool ok = false;
  File f = LittleFS.open(path, "w");
  if (f) {
    size_t wrote = f.write(fb->buf, fb->len);
    f.close();
    ok = (wrote == fb->len);
    if (!ok) {
      Serial.println("Photo write incomplete");
      LittleFS.remove(path);
    }
  } else {
    Serial.println("Photo open for write failed");
  }

  Serial.printf("Photo %s (%u bytes) %s\n", path.c_str(),
                (unsigned)fb->len, ok ? "OK" : "FAIL");

  esp_camera_fb_return(fb);
  stopCamera();
  delay(10);
  startI2C();

  if (ok) {
    time_t now = time(nullptr);
    meta.epoch[slot]    = (now > 1700000000) ? (uint32_t)now : 0;
    meta.distance[slot] = (int16_t)distanceAtTrigger;
    meta.baseline[slot] = (int16_t)baselineDistance;
    meta.nextIndex      = (meta.nextIndex + 1) % MAX_PHOTOS;
    meta.totalCaptures++;
    saveMeta();
    lastBreakDistance = distanceAtTrigger;
    lastBreakEpoch    = (now > 1700000000) ? (uint32_t)now : 0;
  }
  return ok;
}

// ======================= Helpers =========================

String formatUptime(unsigned long ms) {
  unsigned long secs = ms / 1000;
  unsigned long mins = secs / 60;
  unsigned long hrs  = mins / 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
           hrs, mins % 60, secs % 60);
  return String(buf);
}

String formatEpoch(uint32_t epoch) {
  if (epoch < 1700000000) return String("—");
  time_t t = epoch;
  struct tm tmv;
  localtime_r(&t, &tmv);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return String(buf);
}

const char *stateName(SystemState s) {
  switch (s) {
    case STATE_WARMUP:      return "WARMUP";
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_MONITORING:  return "MONITORING";
    case STATE_TRIGGERED:   return "TRIGGERED";
    case STATE_COOLDOWN:    return "COOLDOWN";
  }
  return "UNKNOWN";
}

// ======================= Web: shared CSS ================

const char PAGE_CSS[] PROGMEM = R"CSS(
:root{
  --bg:#050505;--panel:#0b0b0b;--border:#1a1a1a;--border-hi:#262626;
  --fg:#c8c8c8;--dim:#555;--accent:#00ff66;--accent-dim:#00aa44;
  --warn:#ff3344;--info:#00d0ff;--amber:#ffb000;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{background:var(--bg);color:var(--fg);
  font-family:'JetBrains Mono','Fira Code',Menlo,Consolas,monospace;
  min-height:100vh}
body{padding:24px;max-width:1400px;margin:0 auto}
a{color:var(--accent);text-decoration:none}
header{display:flex;align-items:center;justify-content:space-between;
  padding-bottom:16px;border-bottom:1px solid var(--border);margin-bottom:24px}
header h1{font-size:1.3rem;letter-spacing:.2em;color:var(--accent);font-weight:400}
header h1::before{content:'> ';color:var(--accent-dim)}
.state{font-size:.7rem;letter-spacing:.2em;padding:5px 14px;
  border:1px solid var(--border-hi);text-transform:uppercase;color:var(--accent)}
.state.warmup,.state.calibrating{color:var(--info);border-color:var(--info)}
.state.monitoring{color:var(--accent);border-color:var(--accent-dim)}
.state.triggered{color:var(--warn);border-color:var(--warn)}
.state.cooldown{color:var(--amber);border-color:var(--amber)}
main{display:grid;grid-template-columns:minmax(280px,1fr) 2fr;gap:20px}
@media(max-width:760px){main{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--border);padding:18px}
.panel h2{font-size:.65rem;letter-spacing:.25em;color:var(--accent-dim);
  text-transform:uppercase;margin-bottom:14px;font-weight:400}
.panel h2::before{content:'// '}
.row{display:flex;justify-content:space-between;align-items:baseline;
  padding:7px 0;border-bottom:1px dashed #141414;font-size:.82rem}
.row:last-child{border:0}
.label{color:var(--dim)}
.val{color:var(--accent)}
.val.warn{color:var(--warn)}
.val.dim{color:var(--dim)}
.image-wrap{display:flex;align-items:center;justify-content:center;
  min-height:320px;background:#000;border:1px solid var(--border)}
.image-wrap img{width:100%;height:auto;display:block;filter:contrast(1.05) saturate(.95)}
.image-empty{color:var(--dim);font-size:.8rem;letter-spacing:.15em;padding:60px 0}
.caption{font-size:.7rem;color:var(--dim);padding:10px 2px 0;
  display:flex;justify-content:space-between}
.caption .val{color:var(--accent-dim)}
.actions{margin-top:24px;display:flex;gap:12px;flex-wrap:wrap}
.btn{display:inline-block;padding:10px 20px;background:transparent;
  color:var(--accent);border:1px solid var(--accent-dim);font-family:inherit;
  font-size:.75rem;letter-spacing:.18em;text-transform:uppercase;cursor:pointer;
  transition:.15s}
.btn:hover{background:var(--accent);color:var(--bg)}
.btn.ghost{color:var(--dim);border-color:var(--border-hi)}
.btn.ghost:hover{background:var(--border-hi);color:var(--fg)}
footer{margin-top:32px;padding-top:12px;border-top:1px solid var(--border);
  font-size:.65rem;color:var(--dim);letter-spacing:.15em;text-align:center}
footer::before{content:'[ '}footer::after{content:' ]'}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:8px}
.tile{position:relative;border:1px solid var(--border);background:#000;
  aspect-ratio:4/3;overflow:hidden}
.tile img{width:100%;height:100%;object-fit:cover;display:block}
.tile .badge{position:absolute;top:4px;left:4px;font-size:.6rem;color:var(--accent);
  background:rgba(0,0,0,.75);padding:2px 6px;letter-spacing:.1em}
.tile .meta{position:absolute;bottom:0;left:0;right:0;font-size:.6rem;
  color:var(--dim);background:linear-gradient(transparent,rgba(0,0,0,.85));
  padding:14px 6px 5px}
.tile:hover{border-color:var(--accent-dim)}
.empty-gallery{color:var(--dim);padding:40px;text-align:center;font-size:.8rem;
  letter-spacing:.15em;border:1px dashed var(--border-hi)}
.degraded-banner{display:none;background:#0a0800;border:1px solid #332200;
  padding:12px 16px;margin-bottom:20px;font-size:.72rem;letter-spacing:.12em;
  color:var(--amber);position:relative;overflow:hidden}
.degraded-banner.active{display:block}
.degraded-banner::before{content:'WARNING // ';color:#664400}
.degraded-banner .glitch{animation:flk .15s infinite alternate}
@keyframes flk{0%{opacity:1}100%{opacity:.4}}
)CSS";

// ======================= Web: root =====================

void handleRoot() {
  uint32_t count = storedPhotoCount();
  int32_t newest = nthNewestSlot(0);

  String html;
  html.reserve(8000);
  html += F("<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Security System</title><style>");
  html += FPSTR(PAGE_CSS);
  html += F("</style></head><body>"
            "<header><h1>SECURITY SYSTEM</h1>"
            "<div id='state' class='state'>—</div></header>"
            "<div id='degraded' class='degraded-banner'>"
            "sensor readings are <span class='glitch'>no longer accurate</span> "
            "// i2c bus degradation detected // auto-reset attempted // "
            "detection still active — captures may trigger on noise"
            "</div>"
            "<main>"
            "<section class='panel'><h2>Telemetry</h2>"
            "<div class='row'><span class='label'>baseline</span>"
              "<span id='baseline' class='val'>—</span></div>"
            "<div class='row'><span class='label'>last reading</span>"
              "<span id='lastDist' class='val'>—</span></div>"
            "<div class='row'><span class='label'>threshold</span>"
              "<span class='val'>");
  html += TRIGGER_THRESHOLD;
  html += F(" cm</span></div>"
            "<div class='row'><span class='label'>total breaks</span>"
              "<span id='breaks' class='val'>—</span></div>"
            "<div class='row'><span class='label'>last break dist</span>"
              "<span id='lastBreak' class='val'>—</span></div>"
            "<div class='row'><span class='label'>last break time</span>"
              "<span id='lastBreakT' class='val dim'>—</span></div>"
            "<div class='row'><span class='label'>stored photos</span>"
              "<span id='stored' class='val'>");
  html += count;
  html += F("</span></div>"
            "<div class='row'><span class='label'>uptime</span>"
              "<span id='uptime' class='val dim'>—</span></div>"
            "</section>"
            "<section class='panel'><h2>Last Capture</h2>"
            "<div class='image-wrap'>");
  if (newest >= 0) {
    html += F("<img id='liveImg' src='/photo?slot=");
    html += newest;
    html += F("' alt='last capture'>");
  } else {
    html += F("<div class='image-empty'>// NO CAPTURES YET</div>");
  }
  html += F("</div>"
            "<div class='caption'>"
              "<span id='capMeta'>");
  if (newest >= 0) {
    html += F("capture #");
    html += meta.totalCaptures;
    html += F(" · dist ");
    html += meta.distance[newest];
    html += F(" cm");
  }
  html += F("</span><span id='capTime' class='val'>");
  if (newest >= 0) html += formatEpoch(meta.epoch[newest]);
  html += F("</span></div>"
            "</section></main>"
            "<div class='actions'>"
              "<a href='/gallery' class='btn'>Open Gallery</a>"
              "<a href='/recalibrate' class='btn ghost'>Recalibrate</a>"
            "</div>"
            "<footer>tf-luna // esp32-s3 // armed</footer>"
            "<script>"
            "let lastCap=0;"
            "async function tick(){"
              "try{"
                "const r=await fetch('/status');if(!r.ok)return;"
                "const s=await r.json();"
                "const st=document.getElementById('state');"
                "st.textContent=s.state;"
                "st.className='state '+s.state.toLowerCase();"
                "document.getElementById('baseline').textContent=s.baseline?s.baseline+' cm':'—';"
                "document.getElementById('lastDist').textContent=s.lastDist>0?s.lastDist+' cm':'—';"
                "document.getElementById('breaks').textContent=s.breaks;"
                "document.getElementById('lastBreak').textContent=s.lastBreak>0?s.lastBreak+' cm':'—';"
                "document.getElementById('lastBreakT').textContent=s.lastBreakT||'—';"
                "document.getElementById('stored').textContent=s.stored;"
                "document.getElementById('uptime').textContent=s.uptime;"
                "const dg=document.getElementById('degraded');"
                "if(dg)dg.className='degraded-banner'+(s.degraded?' active':'');"
                "const wrap=document.querySelector('.image-wrap');"
                "if(s.totalCaptures>lastCap&&s.newestSlot>=0){"
                  "lastCap=s.totalCaptures;"
                  "let img=document.getElementById('liveImg');"
                  "if(!img){wrap.innerHTML=\"<img id='liveImg'>\";img=document.getElementById('liveImg');}"
                  "img.src='/photo?slot='+s.newestSlot+'&t='+lastCap;"
                  "const cm=document.getElementById('capMeta');"
                  "if(cm)cm.textContent='capture #'+s.totalCaptures+' · dist '+s.lastBreak+' cm';"
                  "const ct=document.getElementById('capTime');"
                  "if(ct)ct.textContent=s.lastBreakT||'';"
                "}"
              "}catch(e){}"
            "}"
            "tick();setInterval(tick,500);"
            "</script>"
            "</body></html>");

  server.send(200, "text/html", html);
}

// ======================= Web: gallery ==================

void handleGallery() {
  uint32_t count = storedPhotoCount();

  String html;
  html.reserve(10000);
  html += F("<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Gallery — Security System</title><style>");
  html += FPSTR(PAGE_CSS);
  html += F("</style></head><body>"
            "<header><h1>SECURITY SYSTEM // GALLERY</h1>"
            "<a href='/' class='btn ghost'>Back</a></header>");

  if (count == 0) {
    html += F("<div class='empty-gallery'>// NO CAPTURES STORED</div>");
  } else {
    html += F("<div class='grid'>");
    for (uint32_t i = 0; i < count; i++) {
      int32_t slot = nthNewestSlot(i);
      if (slot < 0) continue;
      html += F("<a class='tile' href='/photo?slot=");
      html += slot;
      html += F("' target='_blank'>"
                "<img loading='lazy' src='/photo?slot=");
      html += slot;
      html += F("'>"
                "<span class='badge'>#");
      html += (meta.totalCaptures - i);
      html += F("</span>"
                "<div class='meta'>");
      html += meta.distance[slot];
      html += F(" cm · ");
      html += formatEpoch(meta.epoch[slot]);
      html += F("</div></a>");
    }
    html += F("</div>");
  }
  html += F("<footer>");
  html += count;
  html += F(" / ");
  html += MAX_PHOTOS;
  html += F(" slots used</footer></body></html>");
  server.send(200, "text/html", html);
}

// ======================= Web: photo ====================

void handlePhoto() {
  if (!server.hasArg("slot")) { server.send(400, "text/plain", "slot?"); return; }
  int slot = server.arg("slot").toInt();
  if (slot < 0 || slot >= MAX_PHOTOS) { server.send(404, "text/plain", "bad slot"); return; }
  String path = photoPath(slot);
  if (!LittleFS.exists(path)) { server.send(404, "text/plain", "no photo"); return; }
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(500, "text/plain", "open fail"); return; }
  server.sendHeader("Cache-Control", "max-age=3600");
  server.streamFile(f, "image/jpeg");
  f.close();
}

// ======================= Web: image alias ==============

void handleImageLatest() {
  int32_t slot = nthNewestSlot(0);
  if (slot < 0) { server.send(204, "text/plain", "no image"); return; }
  String path = photoPath(slot);
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(404, "text/plain", "no image"); return; }
  server.sendHeader("Cache-Control", "no-cache");
  server.streamFile(f, "image/jpeg");
  f.close();
}

// ======================= Web: status JSON ==============

void handleStatus() {
  int32_t newest = nthNewestSlot(0);
  String json = "{";
  json += "\"state\":\"";    json += stateName(systemState); json += "\",";
  json += "\"baseline\":";   json += baselineDistance; json += ",";
  json += "\"lastDist\":";   json += lastDistance; json += ",";
  json += "\"breaks\":";     json += meta.totalCaptures; json += ",";
  json += "\"lastBreak\":";  json += lastBreakDistance; json += ",";
  json += "\"lastBreakT\":\""; json += formatEpoch(lastBreakEpoch); json += "\",";
  json += "\"stored\":";     json += storedPhotoCount(); json += ",";
  json += "\"newestSlot\":"; json += newest; json += ",";
  json += "\"totalCaptures\":"; json += meta.totalCaptures; json += ",";
  json += "\"degraded\":";   json += degraded ? "true" : "false"; json += ",";
  json += "\"quickResets\":"; json += totalQuickResets; json += ",";
  json += "\"uptime\":\"";   json += formatUptime(millis() - bootMillis); json += "\"";
  json += "}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

// ======================= Web: recalibrate ==============

void handleRecalibrate() {
  calibSum = 0;
  calibCount = 0;
  baselineDistance = 0;
  systemState = STATE_CALIBRATING;
  stateStart = millis();
  server.sendHeader("Location", "/");
  server.send(303);
}

// ======================= Setup ==========================

void setup() {
  Serial.begin(115200);
  delay(400);
  bootMillis = millis();
  Serial.println("\n=== Security System ===");

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS mounted");
    loadMeta();
  }

  // I2C (shares GPIO 8/9 with camera — camera must be off)
  startI2C();

  // Heavy-handed sensor recovery on every boot. Handles a TF-Luna
  // whose I2C address got bumped (e.g. corrupted to 0x20).
  tflunaForceFactoryReset();

  // Wi-Fi
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
    Serial.printf("Web UI: http://%s/\n", WiFi.localIP().toString().c_str());
    // mDNS — reachable at http://security-system.local/
    if (MDNS.begin(MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("mDNS:   http://%s.local/\n", MDNS_HOSTNAME);
    } else {
      Serial.println("mDNS: start failed");
    }
    // NTP (Europe/Madrid)
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
  } else {
    Serial.println(" FAILED - continuing without WiFi");
  }

  // Web routes
  server.on("/",            handleRoot);
  server.on("/gallery",     handleGallery);
  server.on("/photo",       handlePhoto);
  server.on("/image",       handleImageLatest);
  server.on("/status",      handleStatus);
  server.on("/recalibrate", handleRecalibrate);
  server.begin();

  // Enter warmup — web server stays responsive
  systemState = STATE_WARMUP;
  stateStart  = millis();
  Serial.println("Warmup (5s)...");
}

// ======================= Main loop ======================

void loop() {
  server.handleClient();

  unsigned long now = millis();

  switch (systemState) {

    case STATE_WARMUP: {
      if (now - stateStart >= WARMUP_MS) {
        systemState = STATE_CALIBRATING;
        stateStart  = now;
        calibSum    = 0;
        calibCount  = 0;
        Serial.println("Calibrating baseline (10s)...");
      }
      break;
    }

    case STATE_CALIBRATING: {
      static unsigned long lastSample = 0;
      if (now - lastSample >= BASELINE_INTERVAL) {
        lastSample = now;
        int d = tflunaReadDistance();
        if (d > 0) {
          calibSum += d;
          calibCount++;
        }
      }
      if (now - stateStart >= BASELINE_MS) {
        if (calibCount > 0) {
          baselineDistance = calibSum / calibCount;
          Serial.printf("Baseline = %d cm (%d samples)\n",
                        baselineDistance, calibCount);
        } else {
          baselineDistance = 0;
          Serial.println("Baseline: no valid samples");
        }
        systemState = STATE_MONITORING;
        Serial.println("Armed - monitoring");
      }
      break;
    }

    case STATE_MONITORING: {
      static unsigned long lastPoll = 0;
      if (now - lastPoll < POLL_INTERVAL_MS) break;
      lastPoll = now;

      int d = tflunaReadDistance();
      if (d <= 0) break;
      lastDistance = d;

      if (baselineDistance <= 0) break;  // no valid baseline → don't trigger

      int diff = abs(d - baselineDistance);
      if (diff >= TRIGGER_THRESHOLD) {
        Serial.printf("BREAK! dist=%d cm (baseline=%d, diff=%d)\n",
                      d, baselineDistance, diff);
        systemState = STATE_TRIGGERED;
        capturePhoto(d);
        systemState = STATE_MONITORING;
      }
      break;
    }

    case STATE_TRIGGERED:
      // transient — set only inside capturePhoto path
      break;
  }
}
