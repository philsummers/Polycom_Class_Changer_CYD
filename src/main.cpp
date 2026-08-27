/*
  ============================================================================
  LESSON BELL SCHEDULER — for the "Cheap Yellow Display" (ESP32-2432S028R)
  ============================================================================

  WHAT THIS DOES
  - Keeps two INDEPENDENT lists of daily timers ("Schedule A" and "Schedule B"),
    each with a variable number of entries (up to MAX_TIMERS each).
  - When the clock hits a timer's HH:MM, it pulses a GPIO pin for that
    schedule (wire this into a relay/buzzer driver to ring a bell).
  - Full touchscreen UI to add / edit / delete / enable-disable timers,
    per schedule, with no need to reflash to change the schedule.
  - Schedules persist in flash (NVS) across power loss.
  - Time comes from NTP over Wi-Fi (auto-configured via a captive portal),
    with a manual time-set screen as a fallback if there's no Wi-Fi.

  HARDWARE
  Board: ESP32-2432S028R "CYD" (2.8" ILI9341 TFT, 320x240, resistive touch
  via XPT2046). This is the common hobbyist pinout for that board — if your
  particular CYD revision differs, adjust the #defines below and in
  User_Setup.h (see next section).

    TFT (shares HSPI):
      MISO=12  MOSI=13  SCLK=14  CS=15  DC=2  RST=-1 (tied to EN)  BL=21
    Touch XPT2046 (separate SPI bus):
      CLK=25   MOSI=32  MISO=39  CS=33  IRQ=36

  Bell outputs (wire to a relay module / MOSFET / buzzer driver — do NOT
  drive a bell directly off a GPIO):
      BELL_A_PIN = 22
      BELL_B_PIN = 27
  These are two of the few GPIOs broken out on the CYD's header (P3). Check
  your board's silkscreen — revisions vary — and change the #defines if needed.

  REQUIRED LIBRARIES (Arduino Library Manager)
    - TFT_eSPI            by Bodmer
    - XPT2046_Touchscreen  by Paul Stoffregen
    - WiFiManager          by tzapu
  (Preferences and time.h/WiFi are part of the ESP32 core.)

  TFT_eSPI SETUP (one-time, required)
  TFT_eSPI is configured at the LIBRARY level, not per-sketch. After
  installing it, open:
      <Arduino/libraries>/TFT_eSPI/User_Setup.h
  and replace its contents with:

      #define ILI9341_DRIVER
      #define TFT_MISO 12
      #define TFT_MOSI 13
      #define TFT_SCLK 14
      #define TFT_CS   15
      #define TFT_DC    2
      #define TFT_RST  -1
      #define TFT_BL   21
      #define TFT_BACKLIGHT_ON HIGH
      #define SPI_FREQUENCY  40000000
      #define LOAD_GLCD
      #define LOAD_FONT2
      #define LOAD_FONT4
      #define LOAD_FONT6
      #define LOAD_FONT7
      #define LOAD_FONT8
      #define LOAD_GFXFF
      #define SMOOTH_FONT

  BOARD SETTINGS
  Tools > Board: "ESP32 Dev Module" (or "ESP32-WROOM-DA Module"), Flash
  Size 4MB, Partition Scheme "Default 4MB with spiffs".

  FIRST BOOT
  On first boot (or whenever it can't reach saved Wi-Fi), the board opens a
  Wi-Fi access point called "BellScheduler-Setup". Connect a phone/laptop to
  it, a captive portal will pop up (or browse to 192.168.4.1) — pick your
  school Wi-Fi and enter the password. The board then syncs time via NTP.
  If you skip this, use the on-device "Set Time Manually" screen instead.
  ============================================================================
*/

#include <SPIFFS.h>
#include <FS.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebserver.h>

#include <ElegantOTA.h>

// ---------------------------------------------------------------------------
// Pin configuration
// ---------------------------------------------------------------------------
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

#define BELL_A_PIN 22
#define BELL_B_PIN 27
#define BELL_PULSE_MS 1500   // how long the bell output stays HIGH per ring

// Touch calibration (raw ADC range from XPT2046) — adjust if taps land
// noticeably off; easiest way is to print p.x/p.y in getTouchPoint() while
// tapping the four corners.
#define TS_MINX 200
#define TS_MAXX 3800
#define TS_MINY 200
#define TS_MAXY 3800

#define SCREEN_W 320
#define SCREEN_H 240

const char * ntpServer = "pool.ntp.org";
const char * timezone = "GMT0BST,M3.5.0/1,M10.5.0";

AsyncWebServer webserver(80);

// ---------------------------------------------------------------------------
// Globals: display + touch
// ---------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI(HSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

// Touch calibration - rough factory defaults, overwritten by the on-device
// calibration screen (Settings -> Calibrate Touch) and saved to flash.
int16_t tsMinX = 200, tsMaxX = 3800, tsMinY = 200, tsMaxY = 3800;


// Simple color palette
#define COL_BG      TFT_BLACK
#define COL_PANEL   0x18E3   // dark blue-grey
#define COL_BTN     0x2965   // slate
#define COL_BTN_OFF  0x0470   // muted red
#define COL_BTN_ON 0x7803   // green-ish
#define COL_TEXT    TFT_WHITE
#define COL_ACCENT  0xFEA0   // yellow (fits the board!)

#define LCD_BACK_LIGHT_PIN 21
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_12_BIT 12
#define LEDC_BASE_FREQ 5000

// ---------------------------------------------------------------------------
// Polycom
// ---------------------------------------------------------------------------
IPAddress multicastIP(224,0,1,116);
const int port = 5001;

WiFiUDP udp;

const uint8_t channel1 = 0x23;    // Paging Group 1
const uint8_t channel2 = 0x24;    // Paging Group 2

uint32_t serial = 0x00010203;
char caller[13] = "CLASS-CHANGE";

const int FRAME_SAMPLES = 160;   // 20ms at 8kHz
uint8_t prevFrame[FRAME_SAMPLES];
uint8_t currFrame[FRAME_SAMPLES];

uint32_t rtpTimestamp = 0;

File wavFile;

uint8_t pcmu_encode(int16_t pcm)
{
    const uint16_t BIAS = 0x84;
    const uint16_t CLIP = 32635;

    int sign = (pcm >> 8) & 0x80;

    if (sign != 0)
        pcm = -pcm;

    if (pcm > CLIP)
        pcm = CLIP;

    pcm += BIAS;

    int exponent = 7;
    for (int expMask = 0x4000; (pcm & expMask) == 0 && exponent > 0; exponent--, expMask >>= 1);

    int mantissa = (pcm >> ((exponent == 0) ? 4 : (exponent + 3))) & 0x0F;

    uint8_t ulaw = ~(sign | (exponent << 4) | mantissa);

    return ulaw;
}

void sendStartPackets(uint8_t channel)
{
    uint8_t pkt[20];
    memset(pkt,0,sizeof(pkt));

    pkt[0] = 0x0F;
    pkt[1] = channel;

    pkt[2] = (serial>>24)&0xff;
    pkt[3] = (serial>>16)&0xff;
    pkt[4] = (serial>>8)&0xff;
    pkt[5] = serial&0xff;

    pkt[6] = 0x0D;

    memcpy(pkt+7, caller, strlen(caller));

    for(int i=0;i<32;i++)
    {
        udp.beginPacket(multicastIP,port);
        udp.write(pkt,sizeof(pkt));
        udp.endPacket();
        delay(5);
    }

    //logger("Start packets sent");
}

void sendEndPacket(uint8_t channel)
{
    uint8_t pkt[20];
    memset(pkt,0,sizeof(pkt));

    pkt[0] = 0xFF;
    pkt[1] = channel;

    pkt[2] = (serial>>24)&0xff;
    pkt[3] = (serial>>16)&0xff;
    pkt[4] = (serial>>8)&0xff;
    pkt[5] = serial&0xff;

    pkt[6] = 0x0D;

    memcpy(pkt+7, caller, strlen(caller));

    for(int i=0;i<12;i++) {
        udp.beginPacket(multicastIP,port);
        udp.write(pkt,sizeof(pkt));
        udp.endPacket();
        delay(5);
    }

    //logger("End packets sent");
}

void sendAudioPacket(uint8_t* prev, uint8_t* curr, bool first, uint8_t channel)
{
    uint8_t packet[512];
    int idx = 0;

    packet[idx++] = 0x10;
    packet[idx++] = channel;

    packet[idx++] = (serial>>24)&0xff;
    packet[idx++] = (serial>>16)&0xff;
    packet[idx++] = (serial>>8)&0xff;
    packet[idx++] = serial&0xff;

    packet[idx++] = 0x0D;

    memcpy(packet+idx,caller,13);
    idx += 13;

    packet[idx++] = 0x00;
    packet[idx++] = 0x00;

    packet[idx++] = (rtpTimestamp>>24)&0xff;
    packet[idx++] = (rtpTimestamp>>16)&0xff;
    packet[idx++] = (rtpTimestamp>>8)&0xff;
    packet[idx++] = rtpTimestamp&0xff;

    if(!first)
    {
        memcpy(packet+idx, prev, FRAME_SAMPLES);
        idx += FRAME_SAMPLES;
    }

    memcpy(packet+idx, curr, FRAME_SAMPLES);
    idx += FRAME_SAMPLES;

    udp.beginPacket(multicastIP,port);
    udp.write(packet,idx);
    udp.endPacket();
}

bool readFrame(uint8_t* ulawBuf)
{
    int16_t pcm[FRAME_SAMPLES];

    int bytes = wavFile.read((uint8_t*)pcm, FRAME_SAMPLES*2);

    if(bytes < FRAME_SAMPLES*2)
        return false;

    for(int i=0;i<FRAME_SAMPLES;i++)
        ulawBuf[i] = pcmu_encode(pcm[i]);

    return true;
}

void streamWav(uint8_t channel)
{
    bool first=true;

    while(readFrame(currFrame))
    {
        sendAudioPacket(prevFrame,currFrame,first, channel);

        memcpy(prevFrame,currFrame,FRAME_SAMPLES);

        rtpTimestamp += FRAME_SAMPLES;

        first=false;

        delay(20); // 20ms frame
    }
}

void broadcastAudio(uint8_t channel) {
    
    wavFile.seek(44); // skip WAV header

    sendStartPackets(channel);

    streamWav(channel);

    sendEndPacket(channel);

}

void populateSerial() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);

    //logger("Setting Client Serial Number");

    if(ret == ESP_OK) {
        serial = (baseMac[2]<<24)+(baseMac[3]<<16)+(baseMac[4]<<8)+baseMac[5];
        Serial.printf("%X\n",serial);
        //WebSerial.printf("Setting client ID to %X\n",serial);

    } else {
        Serial.println("Failed to read MAC address");
        //WebSerial.println("Failed to read MAC address, using defaults");
    }
}


// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------
#define MAX_TIMERS 30

// days bitmask: bit0=Mon bit1=Tue bit2=Wed bit3=Thu bit4=Fri (no weekend bits)
#define DAY_MON 0x01
#define DAY_TUE 0x02
#define DAY_WED 0x04
#define DAY_THU 0x08
#define DAY_FRI 0x10
#define DAYS_ALL_WEEKDAYS 0x1F

struct TimerEntry {
  uint8_t enabled; // 0/1 (stored as byte for clean NVS blob storage)
  uint8_t hour;
  uint8_t minute;
  uint8_t days;    // bitmask, see DAY_* above
};

TimerEntry scheduleA[MAX_TIMERS];
TimerEntry scheduleB[MAX_TIMERS];
uint8_t countA = 0;
uint8_t countB = 0;
bool scheduleAEnabled = true;
bool scheduleBEnabled = true;
int16_t tzOffsetMinutes = 0; // used only for the NTP gmtOffset_sec param

Preferences prefs;
bool timeSynced = false;
uint8_t lastNTPSyncHour = 25;

// ---------------------------------------------------------------------------
// UI state
// ---------------------------------------------------------------------------
enum Screen { SCR_HOME, SCR_LIST, SCR_EDIT, SCR_SETTINGS, SCR_CALIB };
Screen currentScreen = SCR_HOME;

int activeSchedule = 0; // 0 = A, 1 = B (which schedule LIST/EDIT is working on)
int editIndex = -1;     // -1 = adding a new timer
TimerEntry editBuffer;
int listPage = 0;
const int ROWS_PER_PAGE = 5;

bool touchWasDown = false; // simple debounce so one tap = one action



struct Btn { int16_t x, y, w, h; };

const char* DAY_LABELS[5] = { "Mon", "Tue", "Wed", "Thu", "Fri" };
const uint8_t DAY_BITS[5] = { DAY_MON, DAY_TUE, DAY_WED, DAY_THU, DAY_FRI };
const char DAY_LETTERS[5] = { 'M', 'T', 'W', 'T', 'F' };

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
void loadSchedules() {
  prefs.begin("belltimer", true);
  countA = prefs.getUChar("countA", 0);
  countB = prefs.getUChar("countB", 0);
  scheduleAEnabled = prefs.getBool("enA", true);
  scheduleBEnabled = prefs.getBool("enB", true);
  tzOffsetMinutes = prefs.getShort("tz", 0);
  prefs.getBytes("schedA", scheduleA, sizeof(scheduleA));
  prefs.getBytes("schedB", scheduleB, sizeof(scheduleB));
  tsMinX = prefs.getShort("tsMinX", tsMinX);
  tsMaxX = prefs.getShort("tsMaxX", tsMaxX);
  tsMinY = prefs.getShort("tsMinY", tsMinY);
  tsMaxY = prefs.getShort("tsMaxY", tsMaxY);
  prefs.end();
  if (countA > MAX_TIMERS) countA = 0;
  if (countB > MAX_TIMERS) countB = 0;
}

void saveSchedules() {
  prefs.begin("belltimer", false);
  prefs.putUChar("countA", countA);
  prefs.putUChar("countB", countB);
  prefs.putBool("enA", scheduleAEnabled);
  prefs.putBool("enB", scheduleBEnabled);
  prefs.putShort("tz", tzOffsetMinutes);
  prefs.putBytes("schedA", scheduleA, sizeof(scheduleA));
  prefs.putBytes("schedB", scheduleB, sizeof(scheduleB));
  prefs.end();
}

void saveCalibration() {
  prefs.begin("belltimer", false);
  prefs.putShort("tsMinX", tsMinX);
  prefs.putShort("tsMaxX", tsMaxX);
  prefs.putShort("tsMinY", tsMinY);
  prefs.putShort("tsMaxY", tsMaxY);
  prefs.end();
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  uint32_t duty = (4095 / valueMax) * min(value,valueMax);
  ledcWrite(channel,duty);
}


void sortSchedule(TimerEntry* arr, uint8_t count) {
  // small insertion sort by time-of-day — fine for tens of entries
  for (int i = 1; i < count; i++) {
    TimerEntry key = arr[i];
    int keyKey = key.hour * 60 + key.minute;
    int j = i - 1;
    while (j >= 0 && (arr[j].hour * 60 + arr[j].minute) > keyKey) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

void daysToStr(uint8_t days, char* out) {
  for (int i = 0; i < 5; i++) out[i] = (days & DAY_BITS[i]) ? DAY_LETTERS[i] : '.';
  out[5] = '\0';
}

bool getTouchPoint(int16_t &x, int16_t &y) {
  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();
  x = constrain(map(p.x, TS_MINX, TS_MAXX, 0, SCREEN_W), 0, SCREEN_W - 1);
  y = constrain(map(p.y, TS_MINY, TS_MAXY, 0, SCREEN_H), 0, SCREEN_H - 1);
  //x = constrain(map(p.x, tsMinX, tsMaxX, 0, SCREEN_W), 0, SCREEN_W - 1);
  //y = constrain(map(p.y, tsMinY, tsMaxY, 0, SCREEN_H), 0, SCREEN_H - 1);
  return true;
}

bool hit(Btn b, int16_t x, int16_t y) {
  return x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h;
}

void drawButton(Btn b, const char* label, uint16_t color = COL_BTN) {
  int oldDatum = tft.getTextDatum();
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, color);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  tft.setTextColor(COL_TEXT, color);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, b.x + b.w / 2, b.y + b.h / 2 + 1, 2);
  tft.setTextDatum(oldDatum);
}

bool getNow(struct tm &t) {
  time_t now = time(nullptr);
  if (now < 100000) return false; // clock never set
  localtime_r(&now, &t);
  return true;
}


// find the next enabled timer at/after current time-of-day, only considering
// entries whose day mask includes the day it would land on; returns index or -1
int findNextTimer(TimerEntry* arr, uint8_t count, int curKey, int curWday) {
  int best = -1, bestKey = 999999;
  for (int i = 0; i < count; i++) {
    if (!arr[i].enabled) continue;
    for (int dOff = 0; dOff < 7; dOff++) {
      int wday = (curWday + dOff) % 7;
      if (wday == 0 || wday == 6) continue; // no weekends, ever
      int bitIdx = wday - 1; // Mon=1->0 ... Fri=5->4
      if (!(arr[i].days & DAY_BITS[bitIdx])) continue;
      int k = arr[i].hour * 60 + arr[i].minute;
      int totalKey = dOff * 1440 + k;
      int nowKey = dOff == 0 ? curKey : 0;
      if (dOff == 0 && k < curKey) continue; // already passed today
      if (totalKey < bestKey) { bestKey = totalKey; best = i; }
      break; // this entry's nearest occurrence found for this day offset
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Bell firing (non-blocking pulses)
// ---------------------------------------------------------------------------
bool bellAActive = false; unsigned long bellAOffAt = 0;
bool bellBActive = false; unsigned long bellBOffAt = 0;

void triggerBellA() { digitalWrite(BELL_A_PIN, HIGH); bellAActive = true; bellAOffAt = millis() + BELL_PULSE_MS; broadcastAudio(channel1);}
void triggerBellB() { digitalWrite(BELL_B_PIN, HIGH); bellBActive = true; bellBOffAt = millis() + BELL_PULSE_MS; broadcastAudio(channel2);}

void serviceBells() {
  if (bellAActive && millis() >= bellAOffAt) { digitalWrite(BELL_A_PIN, LOW); bellAActive = false; }
  if (bellBActive && millis() >= bellBOffAt) { digitalWrite(BELL_B_PIN, LOW); bellBActive = false; }
}

void checkSchedules() {
  static int lastMinuteKey = -1;
  struct tm t;
  if (!getNow(t)) return;

  // t.tm_wday: 0=Sunday .. 6=Saturday. No bells on weekends, ever.
  if (t.tm_wday == 0 || t.tm_wday == 6) return;
  int bitIdx = t.tm_wday - 1; // Mon(1)->0 ... Fri(5)->4
  uint8_t todayBit = DAY_BITS[bitIdx];


  int curKey = t.tm_hour * 60 + t.tm_min;
  if (curKey == lastMinuteKey) return; // only evaluate once per minute
  lastMinuteKey = curKey;

  if (scheduleAEnabled) {
    for (int i = 0; i < countA; i++) {
      if (scheduleA[i].enabled && (scheduleA[i].days & todayBit) &&
          (scheduleA[i].hour * 60 + scheduleA[i].minute) == curKey) {
      //if (scheduleA[i].enabled && (scheduleA[i].hour * 60 + scheduleA[i].minute) == curKey) {
        triggerBellA();
        break;
      }
    }
  }
  if (scheduleBEnabled) {
    for (int i = 0; i < countB; i++) {
       if (scheduleB[i].enabled && (scheduleB[i].days & todayBit) &&
          (scheduleB[i].hour * 60 + scheduleB[i].minute) == curKey) {
      //if (scheduleB[i].enabled && (scheduleB[i].hour * 60 + scheduleB[i].minute) == curKey) {      
        triggerBellB();
        break;
      }
    }
  }
}

void updateHomeClock(bool force) {
  static char lastLine1[32] = "";
  static char lastLine2[64] = "";
  struct tm t;
  char line1[32], line2[64];

  if (getNow(t)) {
    strftime(line1, sizeof(line1), "%H:%M:%S   %a %d %b %Y", &t);
  } else {
    snprintf(line1, sizeof(line1), "-- clock not set --");
  }

  int curKey = -1;
  if (getNow(t)) curKey = t.tm_hour * 60 + t.tm_min;
  int nA = curKey >= 0 ? findNextTimer(scheduleA, countA, curKey,t.tm_wday) : -1;
  int nB = curKey >= 0 ? findNextTimer(scheduleB, countB, curKey,t.tm_wday) : -1;
  char nextA[24] = "--:--", nextB[24] = "--:--";
  if (nA >= 0) snprintf(nextA, sizeof(nextA), "%02d:%02d", scheduleA[nA].hour, scheduleA[nA].minute);
  if (nB >= 0) snprintf(nextB, sizeof(nextB), "%02d:%02d", scheduleB[nB].hour, scheduleB[nB].minute);
  snprintf(line2, sizeof(line2), "Next A: %s   Next B: %s", nextA, nextB);

  if (force || strcmp(line1, lastLine1) != 0) {
    tft.fillRect(0, 125, SCREEN_W, 16, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString(line1, SCREEN_W / 2, 125, 4);
    strcpy(lastLine1, line1);
  }
  if (force || strcmp(line2, lastLine2) != 0) {
    tft.fillRect(0, 161, SCREEN_W, 14, COL_BG);
    tft.drawString(line2, SCREEN_W / 2, 161, 2);
    strcpy(lastLine2, line2);
  }
}

// ---------------------------------------------------------------------------
// Screen: HOME
// ---------------------------------------------------------------------------
Btn btnSettings= {10, 200, 90, 32};
Btn btnTestA   = {110, 200, 90, 32};
Btn btnTestB   = {220, 200, 90, 32};

Btn btnBack     = {10, 200, 70, 32};
Btn btnAdd      = {90, 200, 70, 32};
Btn btnPrev     = {170, 200, 60, 32};
Btn btnNext     = {240, 200, 60, 32};
Btn btnToggleEn = {SCREEN_W - 110, 4, 100, 26};
Btn rowBtns[ROWS_PER_PAGE];


void drawHome() {
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("Bell Scheduler", SCREEN_W / 2, 6, 4);

  tft.setTextColor(WiFi.status() == WL_CONNECTED ? TFT_GREEN : TFT_RED, COL_BG);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "WiFi --", SCREEN_W - 6, 6, 2);

  // Panel A
  tft.fillRoundRect(10, 40, 125, 82, 6, COL_PANEL);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_TEXT, COL_PANEL);
  tft.drawString("Schedule A", 20, 48, 2);
  tft.drawString(scheduleAEnabled ? "ENABLED" : "disabled", 20, 68, 2);
  
  char bufA[24];
  snprintf(bufA, sizeof(bufA), "%d timer(s)", countA);
  tft.drawString(bufA, 20, 88, 2);

  // Panel B
  tft.fillRoundRect(165, 40, 125, 82, 6, COL_PANEL);
  tft.drawString("Schedule B", 175, 48, 2);
  tft.drawString(scheduleBEnabled ? "ENABLED" : "disabled", 175, 68, 2);
  char bufB[24];
  snprintf(bufB, sizeof(bufB), "%d timer(s)", countB);
  tft.drawString(bufB, 175, 88, 2);

  drawButton(btnSettings, "Settings");
  drawButton(btnTestA, "Test A");
  drawButton(btnTestB, "Test B");

  updateHomeClock(true);
}

void drawList() {
  TimerEntry* arr = (activeSchedule == 0) ? scheduleA : scheduleB;
  uint8_t count   = (activeSchedule == 0) ? countA : countB;
  bool &enabled   = (activeSchedule == 0) ? scheduleAEnabled : scheduleBEnabled;

  tft.fillScreen(COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  char title[24];
  snprintf(title, sizeof(title), "Schedule %s", activeSchedule == 0 ? "A" : "B");
  tft.drawString(title, 10, 6, 4);
  drawButton(btnToggleEn, enabled ? "ON (tap)" : "OFF (tap)", enabled ? COL_BTN_ON : COL_BTN_OFF);

  int totalPages = max(1, (count + ROWS_PER_PAGE - 1) / ROWS_PER_PAGE);
  if (listPage >= totalPages) listPage = totalPages - 1;
  int startIdx = listPage * ROWS_PER_PAGE;

  int y = 40;
  for (int r = 0; r < ROWS_PER_PAGE; r++) {
    int idx = startIdx + r;
    rowBtns[r] = {10, (int16_t)y, 300, 28};
    if (idx < count) {
      char row[32];
      snprintf(row, sizeof(row), "%02d:%02d   %s", arr[idx].hour, arr[idx].minute,
               arr[idx].enabled ? "" : "(disabled)");
      drawButton(rowBtns[r], row, arr[idx].enabled ? COL_BTN : COL_BTN_OFF);
    } else {
      tft.fillRoundRect(rowBtns[r].x, rowBtns[r].y, rowBtns[r].w, rowBtns[r].h, 6, COL_BG);
    }
    y += 32;
  }

  drawButton(btnBack, "Back");
  drawButton(btnAdd, "+ Add");
  drawButton(btnPrev, "< Prev");
  drawButton(btnNext, "Next >");
}
Btn btnHourUp = {90, 34, 30, 30}, btnHourDn = {20, 34, 30, 30};
Btn btnMinUp  = {270, 34, 30, 30}, btnMinDn  = {200, 34, 30, 30};
Btn dayBtns[5]; // computed in drawEdit()
Btn btnEnToggle = {10, 160, 300, 30};
Btn btnSave = {10, 200, 90, 32}, btnDelete = {110, 200, 90, 32}, btnCancel = {220, 200, 90, 32};

void drawEdit() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString(editIndex == -1 ? "New Timer" : "Edit Timer", SCREEN_W / 2, 2, 2);

  // compact single-row time adjuster: [-] HH [+]   [-] MM [+]
  drawButton(btnHourUp, "+");
  drawButton(btnHourDn, "-");
  drawButton(btnMinUp, "+");
  drawButton(btnMinDn, "-");

  char hm[8];
  snprintf(hm, sizeof(hm), "%02d:%02d", editBuffer.hour, editBuffer.minute);
  tft.fillRect(0, 34, SCREEN_W, 30, COL_BG);
  drawButton(btnHourDn, "-"); drawButton(btnHourUp, "+");
  drawButton(btnMinDn, "-"); drawButton(btnMinUp, "+");
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(hm, SCREEN_W / 2, 49, 4);

  // weekday toggle row
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("Rings on:", 10, 70, 2);
  for (int i = 0; i < 5; i++) {
    dayBtns[i] = { (int16_t)(10 + i * 62), 90, 56, 30 };
    bool on = editBuffer.days & DAY_BITS[i];
    drawButton(dayBtns[i], DAY_LABELS[i], on ? COL_BTN_ON : COL_BTN_OFF);
  }

  drawButton(btnEnToggle, editBuffer.enabled ? "Enabled: ON" : "Enabled: OFF",
             editBuffer.enabled ? COL_BTN_ON : COL_BTN_OFF);

  drawButton(btnSave, "Save");
  if (editIndex != -1) drawButton(btnDelete, "Delete", COL_BTN_OFF);
  drawButton(btnCancel, "Cancel");
}

Btn btnWifiSetup = {10, 32, 300, 30};
Btn btnCalibrate = {10, 66, 300, 30};
Btn btnHrUp2 = {235, 116, 40, 30}, btnHrDn2 = {195, 116, 40, 30};
Btn btnMinUp2 = {150, 116, 40, 30}, btnMinDn2 = {110, 116, 40, 30};
Btn btnApplyTime = {10, 150, 140, 28};
Btn btnSettingsBack = {10, 200, 90, 32};
Btn btnOpenA   = {170, 165, 145, 30};
Btn btnOpenB   = {170, 200, 145, 30};

struct tm manualTime;

void drawSettings() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString("Settings", SCREEN_W / 2, 2, 2);

  drawButton(btnWifiSetup, WiFi.status() == WL_CONNECTED ?
             "WiFi Connected - Reconfigure" : "Set Up WiFi");

  drawButton(btnCalibrate, "Calibrate Touch");

  if (!getNow(manualTime)) {
    manualTime = { 0 };
    manualTime.tm_hour = 8; manualTime.tm_min = 0; manualTime.tm_year = 125; manualTime.tm_mon = 0; manualTime.tm_mday = 1;
  }
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("Manual time (used if no WiFi):", 10, 100, 2);

  char hm[8];
  snprintf(hm, sizeof(hm), "%02d:%02d", manualTime.tm_hour, manualTime.tm_min);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(hm, 60, 131, 4);
  drawButton(btnHrUp2, "+"); drawButton(btnHrDn2, "-");
  drawButton(btnMinUp2, "+"); drawButton(btnMinDn2, "-");
  drawButton(btnApplyTime, "Apply Time");
  drawButton(btnSettingsBack, "Back");

  drawButton(btnOpenA, "Edit A");
  drawButton(btnOpenB, "Edit B");
}

void handleHomeTouch(int16_t x, int16_t y) {  
  if (hit(btnSettings, x, y)) { currentScreen = SCR_SETTINGS; drawSettings(); }
  else if (hit(btnTestA, x, y)) { triggerBellA(); }
  else if (hit(btnTestB, x, y)) { triggerBellB(); }
}
// ---------------------------------------------------------------------------
// Screen: LIST (per schedule)
// ---------------------------------------------------------------------------



void handleListTouch(int16_t x, int16_t y) {
  TimerEntry* arr = (activeSchedule == 0) ? scheduleA : scheduleB;
  uint8_t count   = (activeSchedule == 0) ? countA : countB;
  bool &enabled   = (activeSchedule == 0) ? scheduleAEnabled : scheduleBEnabled;

  if (hit(btnBack, x, y)) { currentScreen = SCR_SETTINGS; drawSettings(); return; }
  if (hit(btnToggleEn, x, y)) { enabled = !enabled; saveSchedules(); drawList(); return; }
  if (hit(btnAdd, x, y)) {
    if (count >= MAX_TIMERS) return; // full
    editIndex = -1;
    editBuffer = { 1, 8, 0, DAYS_ALL_WEEKDAYS };
    currentScreen = SCR_EDIT; drawEdit(); return;
  }
  int totalPages = max(1, (count + ROWS_PER_PAGE - 1) / ROWS_PER_PAGE);
  if (hit(btnPrev, x, y)) { if (listPage > 0) { listPage--; drawList(); } return; }
  if (hit(btnNext, x, y)) { if (listPage < totalPages - 1) { listPage++; drawList(); } return; }

  int startIdx = listPage * ROWS_PER_PAGE;
  for (int r = 0; r < ROWS_PER_PAGE; r++) {
    int idx = startIdx + r;
    if (idx < count && hit(rowBtns[r], x, y)) {
      editIndex = idx;
      editBuffer = arr[idx];
      currentScreen = SCR_EDIT; drawEdit(); return;
    }
  }
}

// ---------------------------------------------------------------------------
// Screen: EDIT
// ---------------------------------------------------------------------------


void handleEditTouch(int16_t x, int16_t y) {
  if (hit(btnHourUp, x, y)) { editBuffer.hour = (editBuffer.hour + 1) % 24; drawEdit(); return; }
  if (hit(btnHourDn, x, y)) { editBuffer.hour = (editBuffer.hour + 23) % 24; drawEdit(); return; }
  if (hit(btnMinUp, x, y)) { editBuffer.minute = (editBuffer.minute + 1) % 60; drawEdit(); return; }
  if (hit(btnMinDn, x, y)) { editBuffer.minute = (editBuffer.minute + 59) % 60; drawEdit(); return; }
  for (int i = 0; i < 5; i++) {
    if (hit(dayBtns[i], x, y)) { editBuffer.days ^= DAY_BITS[i]; drawEdit(); return; }
  }
  if (hit(btnEnToggle, x, y)) { editBuffer.enabled = !editBuffer.enabled; drawEdit(); return; }

  if (hit(btnSave, x, y)) {
    TimerEntry* arr = (activeSchedule == 0) ? scheduleA : scheduleB;
    uint8_t* count  = (activeSchedule == 0) ? &countA : &countB;
    if (editIndex == -1) {
      if (*count < MAX_TIMERS) { arr[*count] = editBuffer; (*count)++; }
    } else {
      arr[editIndex] = editBuffer;
    }
    sortSchedule(arr, *count);
    saveSchedules();
    currentScreen = SCR_LIST; drawList(); return;
  }
  if (editIndex != -1 && hit(btnDelete, x, y)) {
    TimerEntry* arr = (activeSchedule == 0) ? scheduleA : scheduleB;
    uint8_t* count  = (activeSchedule == 0) ? &countA : &countB;
    for (int i = editIndex; i < (*count) - 1; i++) arr[i] = arr[i + 1];
    (*count)--;
    saveSchedules();
    currentScreen = SCR_LIST; drawList(); return;
  }
  if (hit(btnCancel, x, y)) { currentScreen = SCR_LIST; drawList(); return; }
}

// ---------------------------------------------------------------------------
// Screen: SETTINGS
// ---------------------------------------------------------------------------

enum CalibStep { CAL_TL, CAL_BR, CAL_DONE };
CalibStep calibStep = CAL_TL;
int16_t rawTLx, rawTLy, rawBRx, rawBRy;
const int16_t CAL_TL_X = 24, CAL_TL_Y = 24;
const int16_t CAL_BR_X = SCREEN_W - 24, CAL_BR_Y = SCREEN_H - 24;

// ---------------------------------------------------------------------------
// Screen: TOUCH CALIBRATION (2-point)
// ---------------------------------------------------------------------------

void drawCrosshair(int16_t cx, int16_t cy) {
  tft.drawLine(cx - 10, cy, cx + 10, cy, COL_ACCENT);
  tft.drawLine(cx, cy - 10, cx, cy + 10, COL_ACCENT);
  tft.drawCircle(cx, cy, 6, COL_ACCENT);
}

void drawCalib() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString("Touch Calibration", SCREEN_W / 2, 10, 2);
  tft.setTextColor(COL_TEXT, COL_BG);

  if (calibStep == CAL_TL) {
    tft.drawString("Tap the crosshair, top-left", SCREEN_W / 2, 40, 2);
    drawCrosshair(CAL_TL_X, CAL_TL_Y);
  } else if (calibStep == CAL_BR) {
    tft.drawString("Tap the crosshair, bottom-right", SCREEN_W / 2, 40, 2);
    drawCrosshair(CAL_BR_X, CAL_BR_Y);
  } else {
    tft.drawString("Calibration saved!", SCREEN_W / 2, 110, 4);
    tft.drawString("Returning to Settings...", SCREEN_W / 2, 140, 2);
  }
}

void handleCalibTouch() {
  // Reads the RAW (unmapped) touch point directly - calibration has to
  // bypass getTouchPoint(), since that function depends on the calibration
  // values we're currently trying to compute.
  if (!ts.touched()) return;
  TS_Point p = ts.getPoint();

  if (calibStep == CAL_TL) {
    rawTLx = p.x; rawTLy = p.y;
    calibStep = CAL_BR;
    drawCalib();
  } else if (calibStep == CAL_BR) {
    rawBRx = p.x; rawBRy = p.y;
    // works regardless of any axis inversion on a given panel
    tsMinX = min(rawTLx, rawBRx);
    tsMaxX = max(rawTLx, rawBRx);
    tsMinY = min(rawTLy, rawBRy);
    tsMaxY = max(rawTLy, rawBRy);
    saveCalibration();
    calibStep = CAL_DONE;
    drawCalib();
    delay(1200);
    currentScreen = SCR_SETTINGS;
    drawSettings();
  }
}



void handleSettingsTouch(int16_t x, int16_t y) {
  if (hit(btnWifiSetup, x, y)) {
    tft.fillScreen(COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString("Join 'BellScheduler-Setup' WiFi", SCREEN_W / 2, 100, 2);
    tft.drawString("to configure, then wait...", SCREEN_W / 2, 130, 2);
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    wm.startConfigPortal("BellScheduler-Setup");
    if (WiFi.status() == WL_CONNECTED) {
      //configTime(tzOffsetMinutes * 60, 0, "pool.ntp.org", "time.nist.gov");
      configTzTime(timezone,ntpServer);
    }
    drawSettings();
    return;
  }

  if (hit(btnCalibrate, x, y)) {
    calibStep = CAL_TL;
    currentScreen = SCR_CALIB;
    drawCalib();
    return;
  }

  if (hit(btnHrUp2, x, y)) { manualTime.tm_hour = (manualTime.tm_hour + 1) % 24; drawSettings(); return; }
  if (hit(btnHrDn2, x, y)) { manualTime.tm_hour = (manualTime.tm_hour + 23) % 24; drawSettings(); return; }
  if (hit(btnMinUp2, x, y)) { manualTime.tm_min = (manualTime.tm_min + 1) % 60; drawSettings(); return; }
  if (hit(btnMinDn2, x, y)) { manualTime.tm_min = (manualTime.tm_min + 59) % 60; drawSettings(); return; }
  if (hit(btnApplyTime, x, y)) {
    time_t now = time(nullptr);
    struct tm base;
    localtime_r(&now, &base);
    if (now < 100000) { base = manualTime; base.tm_sec = 0; }
    base.tm_hour = manualTime.tm_hour;
    base.tm_min = manualTime.tm_min;
    base.tm_sec = 0;
    time_t applied = mktime(&base);
    struct timeval tv = { applied, 0 };
    settimeofday(&tv, NULL);
    drawSettings();
    return;
  }
  if (hit(btnSettingsBack, x, y)) { currentScreen = SCR_HOME; drawHome(); return; }
  if (hit(btnOpenA, x, y)) { activeSchedule = 0; listPage = 0; currentScreen = SCR_LIST; drawList(); return; }
  if (hit(btnOpenB, x, y)) { activeSchedule = 1; listPage = 0; currentScreen = SCR_LIST; drawList(); return; }
}



// ---------------------------------------------------------------------------
// Touch dispatch
// ---------------------------------------------------------------------------
void handleTouch() {
  int16_t x, y;
  bool down = getTouchPoint(x, y);
  if (down && !touchWasDown) {
    switch (currentScreen) {
      case SCR_HOME:     handleHomeTouch(x, y); break;
      case SCR_LIST:     handleListTouch(x, y); break;
      case SCR_EDIT:     handleEditTouch(x, y); break;
      case SCR_SETTINGS: handleSettingsTouch(x, y); break;
      case SCR_CALIB:    handleCalibTouch(); break;
    }
  }
  touchWasDown = down;
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void onRootRequest(AsyncWebServerRequest *request) {
    request->send(200,"text/plain","OTA");
}

void initWebServer() {
    webserver.on("/", onRootRequest);
    ElegantOTA.begin(&webserver);
    webserver.begin();
}


void setup() {
  Serial.begin(115200);

  pinMode(BELL_A_PIN, OUTPUT); digitalWrite(BELL_A_PIN, LOW);
  pinMode(BELL_B_PIN, OUTPUT); digitalWrite(BELL_B_PIN, LOW);

  tft.init();

#if ESP_IDF_VERSION_MAJOR == 5
  ledcAttach(LCD_BACK_LIGHT_PIN,LEDC_BASE_FREQ,LEDC_TIMER_12_BIT);
#else
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(LCD_BACK_LIGHT_PIN,LEDC_CHANNEL_0);
#endif

  tft.setRotation(1); // landscape, 320x240
  tft.fillScreen(COL_BG);

  ledcAnalogWrite(LEDC_CHANNEL_0, 128);


  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  int w = tft.width();
  int h = tft.height();

  Serial.print("Width:"); Serial.println(w);
  Serial.print("Height:"); Serial.println(h);

  loadSchedules();

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("Connecting WiFi (or setup AP)...", SCREEN_W / 2, SCREEN_H / 2, 2);

  char ver[16];
  snprintf(ver, sizeof(ver), "Firmware: %s", AUTO_VERSION);

  tft.drawString(ver,SCREEN_W/2, (SCREEN_H/2)+30,2);

  WiFiManager wm;
  wm.setConfigPortalTimeout(60); // don't block forever if no WiFi around
  bool ok = wm.autoConnect("BellScheduler-Setup");
  if (ok) {
    configTzTime(timezone,ntpServer);
    struct tm t;
    timeSynced = getLocalTime(&t, 8000);
  }

  populateSerial();

  udp.begin(port);

  if(!SPIFFS.begin(true)) {
        Serial.println("SPIFFS failed");        
        return;
  }

  wavFile = SPIFFS.open("/pager-bell.wav","r");

  if(!wavFile) {
        Serial.println("WAV missing");        
        return;
  }

  initWebServer();

  currentScreen = SCR_HOME;
  drawHome();
}

void loop() {

  struct tm timeinfo;

  // Hourly sync with NTP server
  if (getLocalTime(&timeinfo)) {    
    if ((timeinfo.tm_hour != lastNTPSyncHour) & (timeinfo.tm_min == 0)) {
      Serial.println("Syncing Time");
      //configTime(tzOffsetMinutes * 60, 0, "pool.ntp.org", "time.nist.gov");
      configTzTime(timezone,ntpServer);
      lastNTPSyncHour = timeinfo.tm_hour;
    }
  }

  serviceBells();
  checkSchedules();
  handleTouch();

  if (currentScreen == SCR_HOME) {
    static unsigned long lastClock = 0;
    if (millis() - lastClock > 500) { updateHomeClock(false); lastClock = millis(); }
  }

  ElegantOTA.loop();

  delay(15);
}
