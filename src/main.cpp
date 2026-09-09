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

#include <WiFiManager.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebserver.h>
#include <LittleFS.h>
#include <string.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include <ElegantOTA.h>

#include "polycom.h"

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

// Backlight dimming (idle power-saving / screen-protection). Must match
// TFT_BL in TFT_eSPI's User_Setup.h - this takes over that same pin with
// PWM instead of a plain on/off digitalWrite, so the backlight can be
// smoothly dimmed rather than only switched fully on or off.
#define TFT_BL_PIN 21
#define BACKLIGHT_PWM_CHANNEL 0
#define BACKLIGHT_PWM_FREQ_HZ 5000
#define BACKLIGHT_PWM_RES_BITS 8

#define SCREEN_W 320
#define SCREEN_H 240

// Admin web interface login. These compile-time values are only the
// *factory default* - once someone logs in and changes the password via the
// web page, the new one is stored in flash and these defines are never
// consulted again. Change ADMIN_PASS_DEFAULT before flashing if you'd
// rather not ship with a known default, even temporarily.
#define ADMIN_USER_DEFAULT "admin"
#define ADMIN_PASS_DEFAULT "changeme123"
char adminUser[24];
char adminPass[32];


const uint8_t channel1 = POLYCOM_CHANNEL(10);    // Paging Group 1 - Polycom Group 10
const uint8_t channel2 = POLYCOM_CHANNEL(11);    // Paging Group 2 - Polycom Group 11

const char * ntpServer = "pool.ntp.org";
const char * timezone = "GMT0BST,M3.5.0/1,M10.5.0";

AsyncWebServer webserver(8080);

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
#define COL_BTN_ON  TFT_GREENYELLOW //0x0470   // green-ish
#define COL_BTN_OFF 0x7803   // muted-red
#define COL_TEXT    TFT_WHITE
#define COL_ACCENT  0xFEA0   // yellow (fits the board!)

#define LCD_BACK_LIGHT_PIN 21
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_12_BIT 12
#define LEDC_BASE_FREQ 5000


TFT_eSprite scrnSprite = TFT_eSprite(&tft);


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
enum Screen { SCR_HOME, SCR_LIST, SCR_EDIT, SCR_SETTINGS, SCR_CALIB, SCR_HOL_LIST, SCR_HOL_EDIT, SCR_SERVICE, SCR_SCREENSAVER_CFG, SCR_SCREENSAVER };
Screen currentScreen = SCR_HOME;

int activeSchedule = 0; // 0 = A, 1 = B (which schedule LIST/EDIT is working on)
int editIndex = -1;     // -1 = adding a new timer
TimerEntry editBuffer;
int listPage = 0;
const int ROWS_PER_PAGE = 5;

bool touchWasDown = false; // simple debounce so one tap = one action

// Screensaver: shows a moving clock during idle time on the Home screen,
// wakes on any touch, and also wakes itself early if a bell is coming up
// soon. Declared here (not down in its own section) for the same reason as
// calibStep/holidays above - the Settings screen and the touch dispatcher,
// both earlier in this file, need to reference this state.
bool screensaverEnabled = true;
uint8_t screensaverIdleMinutes = 3;
unsigned long lastActivityMillis = 0;
#define SCREENSAVER_WAKE_LEAD_MIN 5 // wake automatically this many minutes before a due bell

// Backlight dimming shares the same idle timer and wake conditions as the
// screensaver above (any touch, or a bell due soon), but is independently
// switchable - some installs may want dimming without the bouncing clock,
// or vice versa. Declared here for the same forward-reference reasons.
bool dimEnabled = true;
uint8_t dimLevelPercent = 20; // 5-100, applied as backlight PWM duty when idle
bool isDimmed = false;


struct Btn { int16_t x, y, w, h; };

const char* DAY_LABELS[5] = { "Mon", "Tue", "Wed", "Thu", "Fri" };
const uint8_t DAY_BITS[5] = { DAY_MON, DAY_TUE, DAY_WED, DAY_THU, DAY_FRI };
const char DAY_LETTERS[5] = { 'M', 'T', 'W', 'T', 'F' };


// Term dates / holiday overrides: date ranges (inclusive) during which BOTH
// schedules are muted entirely, e.g. half-term, Christmas break, INSET days.
// Declared here (not down in its own section) for the same reason as
// calibStep above - the Settings screen needs to reference it when opening
// the "Term Dates" button, and that code appears earlier in the file.
#define MAX_HOLIDAYS 20
struct HolidayRange {
  uint8_t enabled;
  uint16_t startYear; uint8_t startMonth; uint8_t startDay;
  uint16_t endYear;   uint8_t endMonth;   uint8_t endDay;
};
HolidayRange holidays[MAX_HOLIDAYS];
uint8_t holidayCount = 0;
int holListPage = 0;
int holEditIndex = -1; // -1 = adding a new range
HolidayRange holEditBuffer;
const int HOL_ROWS_PER_PAGE = 5;

void drawWifiIcon(bool force);

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
  holidayCount = prefs.getUChar("holCount", 0);
  prefs.getBytes("holidays", holidays, sizeof(holidays));
  screensaverEnabled = prefs.getBool("ssEn", true);
  screensaverIdleMinutes = prefs.getUChar("ssMin", 3);
  dimEnabled = prefs.getBool("dimEn", true);
  dimLevelPercent = prefs.getUChar("dimPct", 20);
   {
    String au = prefs.getString("aUser", ADMIN_USER_DEFAULT);
    String ap = prefs.getString("aPass", ADMIN_PASS_DEFAULT);
    au.toCharArray(adminUser, sizeof(adminUser));
    ap.toCharArray(adminPass, sizeof(adminPass));
  }
  prefs.end();
  if (countA > MAX_TIMERS) countA = 0;
  if (countB > MAX_TIMERS) countB = 0;
  if (holidayCount > MAX_HOLIDAYS) holidayCount = 0;
  if (screensaverIdleMinutes < 1 || screensaverIdleMinutes > 30) screensaverIdleMinutes = 3;
  if (dimLevelPercent < 5 || dimLevelPercent > 100) dimLevelPercent = 20;
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
  prefs.putUChar("holCount", holidayCount);
  prefs.putBytes("holidays", holidays, sizeof(holidays));
  prefs.putBool("ssEn", screensaverEnabled);
  prefs.putUChar("ssMin", screensaverIdleMinutes);
  prefs.putBool("dimEn", dimEnabled);
  prefs.putUChar("dimPct", dimLevelPercent);
  prefs.putString("aUser", adminUser);
  prefs.putString("aPass", adminPass);
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

// ---- term dates / holiday overrides -----------------------------------
long dateVal(uint16_t y, uint8_t m, uint8_t d) { return (long)y * 10000L + (long)m * 100L + d; }

bool isHolidayToday(struct tm &t) {
  long today = dateVal(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  for (int i = 0; i < holidayCount; i++) {
    if (!holidays[i].enabled) continue;
    long s = dateVal(holidays[i].startYear, holidays[i].startMonth, holidays[i].startDay);
    long e = dateVal(holidays[i].endYear, holidays[i].endMonth, holidays[i].endDay);
    if (today >= s && today <= e) return true;
  }
  return false;
}

uint8_t incMonth(uint8_t m) { return (m % 12) + 1; }
uint8_t decMonth(uint8_t m) { return ((m + 10) % 12) + 1; }
uint8_t incDay(uint8_t d) { return (d % 31) + 1; }
uint8_t decDay(uint8_t d) { return ((d + 29) % 31) + 1; }
uint16_t incYear(uint16_t y) { return y >= 2099 ? 2020 : y + 1; }
uint16_t decYear(uint16_t y) { return y <= 2020 ? 2099 : y - 1; }

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
  int oldDatum = scrnSprite.getTextDatum();
  scrnSprite.fillRoundRect(b.x, b.y, b.w, b.h, 6, color);
  scrnSprite.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  scrnSprite.setTextColor(COL_TEXT, color);
  scrnSprite.setTextDatum(MC_DATUM);
  scrnSprite.drawString(label, b.x + b.w / 2, b.y + b.h / 2 + 1, 2);
  scrnSprite.setTextDatum(oldDatum);
}

void drawButton(Btn b, const char* label, uint16_t color, uint16_t text_color) {
  int oldDatum = scrnSprite.getTextDatum();
  scrnSprite.fillRoundRect(b.x, b.y, b.w, b.h, 6, color);
  scrnSprite.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  scrnSprite.setTextColor(text_color, color);
  scrnSprite.setTextDatum(MC_DATUM);
  scrnSprite.drawString(label, b.x + b.w / 2, b.y + b.h / 2 + 1, 2);
  scrnSprite.setTextDatum(oldDatum);
}

// ---------------------------------------------------------------------------
// WATCHDOG TIMER
// ---------------------------------------------------------------------------
// If the main loop() ever stops coming back around - a library edge case, a
// bug, anything - this device would otherwise sit there silently not
// ringing bells until someone notices and power-cycles it. The ESP32's
// built-in Task Watchdog Timer (TWDT) fixes that: it's fed once per loop()
// iteration, and if WDT_TIMEOUT_SEC passes with no feed, the chip panics
// and reboots itself automatically.
//
// WDT_TIMEOUT_SEC is deliberately generous - every genuinely blocking call
// in normal operation (a remote bell trigger's network timeout, a check-in
// request, an mDNS lookup) finishes in well under a second, so 15s leaves
// huge headroom before this could ever misfire during ordinary use.
//
// One call in this sketch is a legitimate, intentional exception: the
// WiFi reconfigure portal (Settings -> Set Up WiFi) can block for up to
// 180 seconds while you're physically typing a WiFi password into your
// phone. That call explicitly unsubscribes from the watchdog first and
// resubscribes after (search for "esp_task_wdt_delete" below) - the boot
// time WiFi connect in setup() doesn't need the same treatment since the
// watchdog isn't started until after it's already finished.
//
// COMPATIBILITY NOTE: this uses the esp_task_wdt_init(seconds, panic) form
// that's standard on arduino-esp32 core 2.x (still the default/stable
// channel most Arduino IDE installs use). Core 3.x (ESP-IDF 5-based)
// changed that one function's signature to take an esp_task_wdt_config_t
// struct instead and auto-starts its own default watchdog at boot - if
// you're on core 3.x and this fails to compile, replace the init line
// below with esp_task_wdt_reconfigure(&cfg) using that struct; everything
// else here (add/reset/delete) is unchanged across both core versions.
#define WDT_TIMEOUT_SEC 15

void startWatchdog() {
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true); // true = panic (reboot) on timeout, not just warn
  esp_task_wdt_add(NULL);                   // subscribe the current (loop) task
}

// ---------------------------------------------------------------------------
// Icon buttons
// ---------------------------------------------------------------------------
// A handful of simple vector glyphs drawn with plain TFT_eSPI primitives
// (triangles/rects/circles only - no bitmap assets, no filesystem, nothing
// that depends on a specific TFT_eSPI version's font/graphics extras). This
// is a deliberately lighter-weight alternative to pulling in LVGL: a full
// LVGL migration would mean adding a display/touch driver bridge and
// rewriting every screen in this file as LVGL widgets - a large, risky
// rewrite of a ~1700-line sketch for what's fundamentally a cosmetic ask.
// Icons here are reserved for actions that are near-universally recognized
// at a glance (back, add, settings) - destructive or easily-confused actions
// (Save, Delete, Enable/Disable toggles) deliberately keep text labels, since
// misreading an icon matters more when the action isn't reversible.
enum IconType { ICON_BACK, ICON_PLUS, ICON_GEAR, ICON_BELL, ICON_WIFI };

void drawIcon(int16_t cx, int16_t cy, int16_t r, IconType icon, uint16_t fg, uint16_t bg) {
  switch (icon) {
    case ICON_BACK:
      scrnSprite.fillTriangle(cx - r, cy, cx + r * 0.6, cy - r * 0.8, cx + r * 0.6, cy + r * 0.8, fg);
      break;
    case ICON_PLUS: {
      int16_t t = max((int16_t)2, (int16_t)(r / 2));
      scrnSprite.fillRect(cx - r, cy - t / 2, r * 2, t, fg);
      scrnSprite.fillRect(cx - t / 2, cy - r, t, r * 2, fg);
      break;
    }
    case ICON_GEAR: {
      scrnSprite.fillCircle(cx, cy, r, fg);
      const int teeth = 8;
      for (int i = 0; i < teeth; i++) {
        float ang = i * (2.0 * PI / teeth);
        int16_t tx = cx + cos(ang) * r * 1.25;
        int16_t ty = cy + sin(ang) * r * 1.25;
        scrnSprite.fillCircle(tx, ty, r * 0.3, fg);
      }
      scrnSprite.fillCircle(cx, cy, r * 0.4, bg); // punch the center hole through to the button's own background
      break;
    }
    case ICON_BELL: {
      scrnSprite.fillTriangle(cx - r, cy + r * 0.3, cx + r, cy + r * 0.3, cx, cy - r, fg);
      scrnSprite.fillRect(cx - r, cy + r * 0.15, r * 2, r * 0.35, fg);
      scrnSprite.fillCircle(cx, cy + r * 0.65, r * 0.2, fg);
      break;
    }
    case ICON_WIFI: {
      // ascending signal bars rather than the classic radiating arcs - just
      // as recognizable at this size, and only needs fillRect (no drawArc,
      // which behaves slightly differently across TFT_eSPI versions).
      const int bars = 4;
      int16_t barW = max((int16_t)2, (int16_t)(r / 2));
      int16_t gap = 2;
      int16_t totalW = bars * barW + (bars - 1) * gap;
      int16_t startX = cx - totalW / 2;
      int16_t baseY = cy + r;
      //int16_t quality = (2 * (WiFi.RSSI() + 100)) % 4;
      int quality;
      int rssi = WiFi.RSSI();
      if (rssi > -50) {
        quality = 4;
      } else if (rssi > -62) {
        quality = 3;
      } else if (rssi > -68) {
        quality = 2;
      } else if (rssi > -79) {
        quality = 1;
      } else {
        quality = 0;
      }

      for (int i = 0; i < bars; i++) {
        int16_t barH = (r * 0.7) + i * (r * 0.55);
        int16_t bx = startX + i * (barW + gap);
        scrnSprite.fillRect(bx, baseY - barH, barW, barH, (i <= quality) ? fg : TFT_LIGHTGREY);
      }
      break;
    }
  }
}

void drawIconButton(Btn b, IconType icon, uint16_t color = COL_BTN) {
  scrnSprite.fillRoundRect(b.x, b.y, b.w, b.h, 6, color);
  scrnSprite.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  int16_t cx = b.x + b.w / 2, cy = b.y + b.h / 2;
  int16_t r = (min(b.w, b.h) / 2) - 6;
  drawIcon(cx, cy, r, icon, TFT_WHITE, color);
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

// Minutes until the earliest still-upcoming enabled timer scheduled for
// TODAY on either schedule, or -1 if none remain today (weekend, holiday
// override, or nothing left on the clock). Deliberately only looks at
// today - not tomorrow - since its only use is deciding whether to keep the
// screensaver off; a small blind spot right around midnight for a very
// early first bell is an acceptable trade-off for the simplicity here.
int minutesUntilNextBellToday() {
  struct tm t;
  if (!getNow(t)) return -1;
  if (isHolidayToday(t)) return -1;
  if (t.tm_wday == 0 || t.tm_wday == 6) return -1;
  uint8_t todayBit = DAY_BITS[t.tm_wday - 1];
  int curKey = t.tm_hour * 60 + t.tm_min;
  int best = -1;
  if (scheduleAEnabled) {
    for (int i = 0; i < countA; i++) {
      if (scheduleA[i].enabled && (scheduleA[i].days & todayBit)) {
        int k = scheduleA[i].hour * 60 + scheduleA[i].minute;
        if (k >= curKey && (best == -1 || k < best)) best = k;
      }
    }
  }
  if (scheduleBEnabled) {
    for (int i = 0; i < countB; i++) {
      if (scheduleB[i].enabled && (scheduleB[i].days & todayBit)) {
        int k = scheduleB[i].hour * 60 + scheduleB[i].minute;
        if (k >= curKey && (best == -1 || k < best)) best = k;
      }
    }
  }
  return best == -1 ? -1 : (best - curKey);
}

bool bellImminent() {
  int m = minutesUntilNextBellToday();
  return m >= 0 && m <= SCREENSAVER_WAKE_LEAD_MIN;
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
        triggerBellA();
        break;
      }
    }
  }
  if (scheduleBEnabled) {
    for (int i = 0; i < countB; i++) {
       if (scheduleB[i].enabled && (scheduleB[i].days & todayBit) &&
          (scheduleB[i].hour * 60 + scheduleB[i].minute) == curKey) {
        triggerBellB();
        break;
      }
    }
  }
}

void updateHomeClock(bool force) {
  static char lastLine1[32] = "";
  static char lastLine2[64] = "";
  static bool lastHoliday = false;

  struct tm t;
  char line1[32], line2[64];

  drawWifiIcon(true);

  if (getNow(t)) {
    strftime(line1, sizeof(line1), "%a %d %b %Y   %H:%M:%S", &t);
  } else {
    snprintf(line1, sizeof(line1), "-- clock not set --");
  }

  bool onHoliday = false;
  
  int nextA = -1, nextB = -1;
  if (getNow(t)) {
    onHoliday = isHolidayToday(t);
    int curKey = t.tm_hour * 60 + t.tm_min;
    nextA = findNextTimer(scheduleA, countA, curKey, t.tm_wday);
    nextB = findNextTimer(scheduleB, countB, curKey, t.tm_wday);
  }
  if (onHoliday) {
    snprintf(line2, sizeof(line2), "TERM DATE OVERRIDE - no bells today");
  } else {
    char nextAStr[24] = "--:--", nextBStr[24] = "--:--";
    if (nextA >= 0) snprintf(nextAStr, sizeof(nextAStr), "%02d:%02d", scheduleA[nextA].hour, scheduleA[nextA].minute);
    if (nextB >= 0) snprintf(nextBStr, sizeof(nextBStr), "%02d:%02d", scheduleB[nextB].hour, scheduleB[nextB].minute);
    snprintf(line2, sizeof(line2), "Next A: %s   Next B: %s", nextAStr, nextBStr);
  }

  if (force || strcmp(line1, lastLine1) != 0) {
    scrnSprite.fillRect(0, 125, SCREEN_W, 16, COL_BG);
    scrnSprite.setTextDatum(TC_DATUM);
    scrnSprite.setTextColor(COL_TEXT, COL_BG);
    scrnSprite.drawString(line1, SCREEN_W / 2, 125, 4);
    strcpy(lastLine1, line1);
  }
  if (force || strcmp(line2, lastLine2) != 0 || onHoliday != lastHoliday) {
    scrnSprite.fillRect(0, 161, SCREEN_W, 14, COL_BG);
    scrnSprite.setTextColor(onHoliday ? TFT_RED : COL_TEXT, COL_BG);
    scrnSprite.drawString(line2, SCREEN_W / 2, 161, 2);
    strcpy(lastLine2, line2);
    lastHoliday = onHoliday;
  }
}

// ---------------------------------------------------------------------------
// ADMIN WEB INTERFACE
// ---------------------------------------------------------------------------
// A second, separate web server on port 80 (the check-in server above stays
// on 8090 and unauthenticated, since it's machine-to-machine on a trusted
// LAN - this one is human-facing and password protected with HTTP Basic
// Auth). Deliberately scoped to what's most useful to reach remotely: the
// two timer schedules (the actual ask), a live status readout, bell tests
// for wiring checks, and the ability to change the admin password. Term
// dates, remote-unit assignment, and screensaver settings stay device-only
// for now - see the extension notes at the end of this file for that.
#define ADMIN_PORT 80
WebServer adminServer(ADMIN_PORT);

bool checkAdminAuth() {
  if (!adminServer.authenticate(adminUser, adminPass)) {
    adminServer.requestAuthentication();
    return false;
  }
  return true;
}

// The web page's HTML/CSS/JS used to be embedded here as a PROGMEM string,
// which counts against the sketch's compiled program-storage size. It now
// lives as plain files (index.html, style.css, app.js) on the ESP32's
// LittleFS partition instead - a separate flash region that isn't part of
// the firmware binary at all, so none of it counts toward "Sketch uses X%
// of program storage space." This is the only approach that actually
// reduces that figure; splitting the .ino into more tabs/files would not,
// since the Arduino IDE compiles everything in a sketch folder together
// into one binary regardless of how many files it's split across.
//
// SETTING THIS UP (one-time, separate from flashing the sketch)
// 1. Put index.html, style.css, and app.js in a folder named "data" next to
//    this .ino file (Arduino requires that exact folder name and location).
// 2. Upload that folder to the device's filesystem partition - this is a
//    SEPARATE step from uploading the sketch:
//      - Arduino IDE 1.8.x: install the "ESP32 Sketch Data Upload" tool,
//        then Tools -> ESP32 Sketch Data Upload.
//      - Arduino IDE 2.x: that old tool doesn't work the same way - install
//        the community "arduino-littlefs-upload" plugin instead (search
//        that name), which adds the same option to the command palette.
//      - PlatformIO: use "Upload Filesystem Image" - no plugin needed.
// 3. The existing "Default 4MB with spiffs" partition scheme (already
//    noted in BOARD SETTINGS above) has a filesystem partition big enough
//    for these files with plenty of room to spare - no change needed there.
// If you forget this step, the web page will show a clear message telling
// you to do it, rather than a bare 404.
bool serveAdminFile(const char* path, const char* contentType) {
  if (!checkAdminAuth()) return true; // auth challenge already sent
  if (!SPIFFS.exists(path)) {
    adminServer.send(500, "text/plain",
      String("Web UI file not found on LittleFS: ") + path +
      "\n\nUpload the sketch's 'data' folder to the device's filesystem "
      "partition (Tools > ESP32 Sketch Data Upload on Arduino IDE 1.8.x, "
      "or the arduino-littlefs-upload plugin on Arduino IDE 2.x). See the "
      "comment above serveAdminFile() in bell_scheduler.ino for details.");
    return true;
  }
  File f = SPIFFS.open(path, "r");
  adminServer.streamFile(f, contentType);
  f.close();
  return true;
}

void handleAdminRoot() { serveAdminFile("/index.html", "text/html"); }
void handleAdminCss()  { serveAdminFile("/style.css", "text/css"); }
void handleAdminJs()   { serveAdminFile("/app.js", "application/javascript"); }

void handleApiStatus() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(512);
  JsonDocument doc;
  struct tm t;
  char timeStr[40] = "clock not set";
  if (getNow(t)) strftime(timeStr, sizeof(timeStr), "%H:%M:%S %a %d %b %Y", &t);
  doc["time"] = timeStr;
  unsigned long upSec = millis() / 1000;
  char upStr[24];
  snprintf(upStr, sizeof(upStr), "%luh %02lum", upSec / 3600, (upSec / 60) % 60);
  doc["uptime"] = upStr;
  doc["heap"] = (unsigned long)ESP.getFreeHeap();
  doc["wifi"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String("not connected");
  doc["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  doc["ip"] = WiFi.localIP().toString();
  String out;
  serializeJson(doc, out);
  adminServer.send(200, "application/json", out);
}

void handleApiScheduleGet() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(8192);
  JsonDocument doc;
  doc["enabledA"] = scheduleAEnabled;
  doc["enabledB"] = scheduleBEnabled;
  JsonArray a = doc.createNestedArray("a");
  for (int i = 0; i < countA; i++) {
    JsonObject o = a.createNestedObject();
    o["i"] = i; o["h"] = scheduleA[i].hour; o["m"] = scheduleA[i].minute;
    o["en"] = (bool)scheduleA[i].enabled; o["days"] = scheduleA[i].days;
  }
  JsonArray b = doc.createNestedArray("b");
  for (int i = 0; i < countB; i++) {
    JsonObject o = b.createNestedObject();
    o["i"] = i; o["h"] = scheduleB[i].hour; o["m"] = scheduleB[i].minute;
    o["en"] = (bool)scheduleB[i].enabled; o["days"] = scheduleB[i].days;
  }
  String out;
  serializeJson(doc, out);
  adminServer.send(200, "application/json", out);
}

void handleApiTimerPost() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(512);
  JsonDocument doc;
  if (deserializeJson(doc, adminServer.arg("plain"))) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
    return;
  }
  const char* schedStr = doc["sched"] | "A";
  int idx = doc["i"] | -1;
  int h = constrain((int)(doc["h"] | 8), 0, 23);
  int m = constrain((int)(doc["m"] | 0), 0, 59);
  bool en = doc["en"] | true;
  uint8_t days = (uint8_t)(doc["days"] | DAYS_ALL_WEEKDAYS) & 0x1F;

  TimerEntry* arr = (schedStr[0] == 'A') ? scheduleA : scheduleB;
  uint8_t* count = (schedStr[0] == 'A') ? &countA : &countB;
  TimerEntry te = { (uint8_t)(en ? 1 : 0), (uint8_t)h, (uint8_t)m, days };

  if (idx < 0) {
    if (*count >= MAX_TIMERS) {
      adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"schedule is full\"}");
      return;
    }
    arr[*count] = te; (*count)++;
  } else {
    if (idx >= *count) {
      adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad index\"}");
      return;
    }
    arr[idx] = te;
  }
  sortSchedule(arr, *count);
  saveSchedules();
  adminServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiTimerDelete() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(256);
  JsonDocument doc;
  if (deserializeJson(doc, adminServer.arg("plain"))) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
    return;
  }
  const char* schedStr = doc["sched"] | "A";
  int idx = doc["i"] | -1;
  TimerEntry* arr = (schedStr[0] == 'A') ? scheduleA : scheduleB;
  uint8_t* count = (schedStr[0] == 'A') ? &countA : &countB;
  if (idx < 0 || idx >= *count) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad index\"}");
    return;
  }
  for (int i = idx; i < (*count) - 1; i++) arr[i] = arr[i + 1];
  (*count)--;
  saveSchedules();
  adminServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiScheduleEnable() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(128);
  JsonDocument doc;
  if (deserializeJson(doc, adminServer.arg("plain"))) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
    return;
  }
  const char* schedStr = doc["sched"] | "A";
  bool en = doc["en"] | true;
  if (schedStr[0] == 'A') scheduleAEnabled = en; else scheduleBEnabled = en;
  saveSchedules();
  adminServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiBellTest() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(128);
  JsonDocument doc;
  if (deserializeJson(doc, adminServer.arg("plain"))) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
    return;
  }
  const char* schedStr = doc["sched"] | "A";
  if (schedStr[0] == 'A') triggerBellA(); else triggerBellB();
  adminServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPassword() {
  if (!checkAdminAuth()) return;
  //DynamicJsonDocument doc(256);
  JsonDocument doc;
  if (deserializeJson(doc, adminServer.arg("plain"))) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
    return;
  }
  const char* oldPass = doc["oldPass"] | "";
  const char* newPass = doc["newPass"] | "";
  if (strcmp(oldPass, adminPass) != 0) {
    adminServer.send(403, "application/json", "{\"ok\":false,\"error\":\"current password is incorrect\"}");
    return;
  }
  if (strlen(newPass) < 4) {
    adminServer.send(400, "application/json", "{\"ok\":false,\"error\":\"new password must be at least 4 characters\"}");
    return;
  }
  strncpy(adminPass, newPass, sizeof(adminPass) - 1);
  adminPass[sizeof(adminPass) - 1] = '\0';
  saveSchedules();
  adminServer.send(200, "application/json", "{\"ok\":true}");
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


void drawWifiIcon(bool force) {
  static int lastState = -1; // -1 = never drawn, 0 = disconnected, 1 = connected
  int state = (WiFi.status() == WL_CONNECTED) ? 1 : 0;
  if (!force && state == lastState) return; // avoid needless redraws every poll
  lastState = state;
  scrnSprite.fillRect(SCREEN_W - 32, 2, 30, 26, COL_BG); // clear the corner first - bars vary in height
  drawIcon(SCREEN_W - 18, 16, 10, ICON_WIFI, state ? TFT_GREEN : TFT_RED, COL_BG);
}

void drawHome() {
  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  scrnSprite.setTextDatum(TC_DATUM);
  scrnSprite.drawString("Bell Scheduler", SCREEN_W / 2, 6, 4);

  //tft.setTextColor(WiFi.status() == WL_CONNECTED ? TFT_GREEN : TFT_RED, COL_BG);
  //tft.setTextDatum(TR_DATUM);
  //tft.drawString(WiFi.status() == WL_CONNECTED ? "WiFi OK" : "WiFi --", SCREEN_W - 6, 6, 2);
  drawWifiIcon(true);

  // Panel A
  scrnSprite.fillRoundRect(10, 40, 125, 68, 6, scheduleAEnabled ? COL_BTN_ON : COL_BTN_OFF);
  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(scheduleAEnabled ? TFT_BLACK : TFT_WHITE, scheduleAEnabled ? COL_BTN_ON : COL_BTN_OFF);
  scrnSprite.drawString("Schedule A", 20, 48, 2);
  scrnSprite.drawString(scheduleAEnabled ? "ENABLED" : "disabled", 20, 68, 2);
  
  char bufA[24];
  snprintf(bufA, sizeof(bufA), "%d timer(s)", countA);
  scrnSprite.drawString(bufA, 20, 88, 2);

  // Panel B
  scrnSprite.fillRoundRect(185, 40, 125, 68, 6, scheduleBEnabled ? COL_BTN_ON : COL_BTN_OFF);
  scrnSprite.setTextColor(scheduleBEnabled ? TFT_BLACK : TFT_WHITE, scheduleBEnabled ? COL_BTN_ON : COL_BTN_OFF);
  scrnSprite.drawString("Schedule B", 195, 48, 2);
  scrnSprite.drawString(scheduleBEnabled ? "ENABLED" : "disabled", 195, 68, 2);
  char bufB[24];
  snprintf(bufB, sizeof(bufB), "%d timer(s)", countB);
  scrnSprite.drawString(bufB, 195, 88, 2);

  //drawButton(btnSettings, "Settings");
  drawIconButton(btnSettings,ICON_GEAR);
  //drawIconButton(btnTestA,ICON_BELL);
  drawButton(btnTestA, "Test A");
  drawButton(btnTestB, "Test B");

  updateHomeClock(true);
  //scrnSprite.pushSprite(0,0);

}

void drawList() {
  TimerEntry* arr = (activeSchedule == 0) ? scheduleA : scheduleB;
  uint8_t count   = (activeSchedule == 0) ? countA : countB;
  bool &enabled   = (activeSchedule == 0) ? scheduleAEnabled : scheduleBEnabled;

  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  char title[24];
  snprintf(title, sizeof(title), "Schedule %s", activeSchedule == 0 ? "A" : "B");
  scrnSprite.drawString(title, 10, 6, 4);
  drawButton(btnToggleEn, enabled ? "ON (tap)" : "OFF (tap)", enabled ? COL_BTN_ON : COL_BTN_OFF, enabled ? TFT_BLACK : TFT_WHITE);

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
      scrnSprite.fillRoundRect(rowBtns[r].x, rowBtns[r].y, rowBtns[r].w, rowBtns[r].h, 6, COL_BG);
    }
    y += 32;
  }

  //drawButton(btnBack, "Back");
  drawIconButton(btnBack, ICON_BACK);
  //drawButton(btnAdd, "+ Add");
  drawIconButton(btnAdd, ICON_PLUS);
  drawButton(btnPrev, "< Prev");
  drawButton(btnNext, "Next >");
}
Btn btnHourUp = {90, 34, 30, 30}, btnHourDn = {20, 34, 30, 30};
Btn btnMinUp  = {270, 34, 30, 30}, btnMinDn  = {200, 34, 30, 30};
Btn dayBtns[5]; // computed in drawEdit()
Btn btnEnToggle = {10, 160, 300, 30};
Btn btnSave = {10, 200, 90, 32}, btnDelete = {110, 200, 90, 32}, btnCancel = {220, 200, 90, 32};

void drawEdit() {
  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextDatum(TC_DATUM);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  scrnSprite.drawString(editIndex == -1 ? "New Timer" : "Edit Timer", SCREEN_W / 2, 2, 2);

  // compact single-row time adjuster: [-] HH [+]   [-] MM [+]
  drawButton(btnHourUp, "+");
  drawButton(btnHourDn, "-");
  drawButton(btnMinUp, "+");
  drawButton(btnMinDn, "-");

  char hm[8];
  snprintf(hm, sizeof(hm), "%02d:%02d", editBuffer.hour, editBuffer.minute);
  scrnSprite.fillRect(0, 34, SCREEN_W, 30, COL_BG);
  drawButton(btnHourDn, "-"); drawButton(btnHourUp, "+");
  drawButton(btnMinDn, "-"); drawButton(btnMinUp, "+");
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  scrnSprite.setTextDatum(MC_DATUM);
  scrnSprite.drawString(hm, SCREEN_W / 2, 49, 4);

  // weekday toggle row
  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  scrnSprite.drawString("Rings on:", 10, 70, 2);
  for (int i = 0; i < 5; i++) {
    dayBtns[i] = { (int16_t)(10 + i * 62), 90, 56, 30 };
    bool on = editBuffer.days & DAY_BITS[i];
    drawButton(dayBtns[i], DAY_LABELS[i], on ? COL_BTN_ON : COL_BTN_OFF, on ? TFT_BLACK : TFT_WHITE);
  }

  drawButton(btnEnToggle, editBuffer.enabled ? "Enabled: ON" : "Enabled: OFF",
             editBuffer.enabled ? COL_BTN_ON : COL_BTN_OFF, editBuffer.enabled ? TFT_BLACK : TFT_WHITE);

  drawButton(btnSave, "Save");
  if (editIndex != -1) drawButton(btnDelete, "Delete", COL_BTN_OFF);
  //drawButton(btnCancel, "Cancel");
  drawIconButton(btnCancel, ICON_BACK);
}

Btn btnWifiSetup = {10, 32, 300, 30};
Btn btnCalibrate = {10, 66, 300, 30};
Btn btnScreensaver = {10, 100, 300, 30};
Btn btnHrUp2 = {235, 116, 40, 30}, btnHrDn2 = {195, 116, 40, 30};
Btn btnMinUp2 = {150, 116, 40, 30}, btnMinDn2 = {110, 116, 40, 30};
Btn btnApplyTime = {10, 150, 140, 28};
Btn btnSettingsBack = {10, 200, 90, 32};
Btn btnOpenA   = {170, 165, 145, 30};
Btn btnOpenB   = {170, 200, 145, 30};

struct tm manualTime;

void drawSettings() {
  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextDatum(TC_DATUM);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  scrnSprite.drawString("Settings", SCREEN_W / 2, 2, 2);

  drawButton(btnWifiSetup, WiFi.status() == WL_CONNECTED ?
             "WiFi Connected - Reconfigure" : "Set Up WiFi");

  drawButton(btnCalibrate, "Calibrate Touch");
  drawButton(btnScreensaver, "Screensaver");

 /* if (!getNow(manualTime)) {
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
  //drawButton(btnSettingsBack, "Back");
*/

  drawIconButton(btnSettingsBack, ICON_BACK);

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
// Screen: SCREENSAVER SETTINGS
// ---------------------------------------------------------------------------
Btn btnSsEnable = {10, 24, 300, 24};
Btn btnSsMinusMin = {90, 66, 50, 24}, btnSsPlusMin = {240, 66, 50, 24};
Btn btnDimEnable = {10, 96, 300, 24};
Btn btnDimMinus = {90, 138, 50, 24}, btnDimPlus = {240, 138, 50, 24};
Btn btnSsBack = {10, 200, 90, 32};

void drawScreensaverSettings() {
  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextDatum(TC_DATUM);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  scrnSprite.drawString("Screensaver", SCREEN_W / 2, 2, 2);

  drawButton(btnSsEnable, screensaverEnabled ? "Screensaver: ON" : "Screensaver: OFF",
             screensaverEnabled ? COL_BTN_ON : COL_BTN_OFF, screensaverEnabled ? TFT_BLACK : TFT_WHITE);

  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  scrnSprite.drawString("Idle timeout:", 10, 54, 2);
  drawButton(btnSsMinusMin, "-");
  drawButton(btnSsPlusMin, "+");
  char mins[16];
  snprintf(mins, sizeof(mins), "%d min", screensaverIdleMinutes);
  scrnSprite.setTextDatum(MC_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  scrnSprite.drawString(mins, 185, 78, 4);

  drawButton(btnDimEnable, dimEnabled ? "Dim Backlight: ON" : "Dim Backlight: OFF",
             dimEnabled ? COL_BTN_ON : COL_BTN_OFF, screensaverEnabled ? TFT_BLACK : TFT_WHITE);
  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  scrnSprite.drawString("Dim level:", 10, 126, 2);
  drawButton(btnDimMinus, "-");
  drawButton(btnDimPlus, "+");
  char pct[16];
  snprintf(pct, sizeof(pct), "%d%%", dimLevelPercent);
  scrnSprite.setTextDatum(MC_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  scrnSprite.drawString(pct, 185, 150, 4);

  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  char note[70];
  snprintf(note, sizeof(note), "Both share the idle timeout above; wake on touch or %d min before a bell.", SCREENSAVER_WAKE_LEAD_MIN);
  scrnSprite.drawString(note, 10, 172, 1);

  //drawButton(btnSsBack, "Back");
  drawIconButton(btnSsBack,ICON_BACK);
}

void handleScreensaverSettingsTouch(int16_t x, int16_t y) {
  if (hit(btnSsEnable, x, y)) { screensaverEnabled = !screensaverEnabled; saveSchedules(); drawScreensaverSettings(); return; }
  if (hit(btnSsMinusMin, x, y)) {
    if (screensaverIdleMinutes > 1) screensaverIdleMinutes--;
    saveSchedules(); drawScreensaverSettings(); return;
  }
  if (hit(btnSsPlusMin, x, y)) {
    if (screensaverIdleMinutes < 30) screensaverIdleMinutes++;
    saveSchedules(); drawScreensaverSettings(); return;
  }
    if (hit(btnDimEnable, x, y)) {
    dimEnabled = !dimEnabled;
    saveSchedules(); drawScreensaverSettings(); return;
  }
  if (hit(btnDimMinus, x, y)) {
    if (dimLevelPercent > 5) dimLevelPercent -= 5;
    saveSchedules(); drawScreensaverSettings(); return;
  }
  if (hit(btnDimPlus, x, y)) {
    if (dimLevelPercent < 100) dimLevelPercent += 5;
    saveSchedules(); drawScreensaverSettings(); return;
  }
  if (hit(btnSsBack, x, y)) { currentScreen = SCR_SETTINGS; drawSettings(); return; }
}



// ---------------------------------------------------------------------------
// Screen: TOUCH CALIBRATION (2-point)
// ---------------------------------------------------------------------------

void drawCrosshair(int16_t cx, int16_t cy) {
  scrnSprite.drawLine(cx - 10, cy, cx + 10, cy, COL_ACCENT);
  scrnSprite.drawLine(cx, cy - 10, cx, cy + 10, COL_ACCENT);
  scrnSprite.drawCircle(cx, cy, 6, COL_ACCENT);
}

void drawCalib() {
  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextDatum(TC_DATUM);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  scrnSprite.drawString("Touch Calibration", SCREEN_W / 2, 10, 2);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);

  if (calibStep == CAL_TL) {
    scrnSprite.drawString("Tap the crosshair, top-left", SCREEN_W / 2, 40, 2);
    drawCrosshair(CAL_TL_X, CAL_TL_Y);
  } else if (calibStep == CAL_BR) {
    scrnSprite.drawString("Tap the crosshair, bottom-right", SCREEN_W / 2, 40, 2);
    drawCrosshair(CAL_BR_X, CAL_BR_Y);
  } else {
    scrnSprite.drawString("Calibration saved!", SCREEN_W / 2, 110, 4);
    scrnSprite.drawString("Returning to Settings...", SCREEN_W / 2, 140, 2);
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
    scrnSprite.fillSprite(COL_BG);
    scrnSprite.setTextDatum(MC_DATUM);
    scrnSprite.setTextColor(COL_TEXT, COL_BG);
    scrnSprite.drawString("Join 'BellScheduler-Setup' WiFi", SCREEN_W / 2, 100, 2);
    scrnSprite.drawString("to configure, then wait...", SCREEN_W / 2, 130, 2);
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    esp_task_wdt_delete(NULL);
    wm.startConfigPortal("BellScheduler-Setup");
    esp_task_wdt_add(NULL);
    if (WiFi.status() == WL_CONNECTED) {
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

  if (hit(btnScreensaver, x, y)) {
    currentScreen = SCR_SCREENSAVER_CFG;
    drawScreensaverSettings();
    return;
  }

  /*
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
    */

  if (hit(btnSettingsBack, x, y)) { currentScreen = SCR_HOME; drawHome(); return; }
  if (hit(btnOpenA, x, y)) { activeSchedule = 0; listPage = 0; currentScreen = SCR_LIST; drawList(); return; }
  if (hit(btnOpenB, x, y)) { activeSchedule = 1; listPage = 0; currentScreen = SCR_LIST; drawList(); return; }
}

// ---------------------------------------------------------------------------
// Screen: SERVICE MENU (hidden - installer/diagnostics only)
// ---------------------------------------------------------------------------
// Not reachable from any visible button. Opened by pressing and holding the
// empty top-left corner of the Home screen (an invisible hotspot) for
// SERVICE_HOLD_MS. This keeps it out of the way of normal end-user staff
// while still being easy for whoever installed the unit to find again.
#define SERVICE_HOLD_MS 1500
Btn svcHotspot = {0, 0, 50, 40}; // invisible - top-left corner of Home screen only
unsigned long svcHoldStart = 0;

Btn btnSvcTestA = {10, 125, 145, 28}, btnSvcTestB = {165, 125, 145, 28};
Btn btnSvcRestart = {10, 159, 145, 28}, btnSvcReset = {165, 159, 145, 28};
Btn btnSvcBack = {10, 200, 90, 32};

bool resetArmed = false;
unsigned long resetArmedAt = 0;
#define RESET_ARM_WINDOW_MS 4000

void drawServiceInfo() {
  scrnSprite.fillRect(0, 30, SCREEN_W, 82, COL_BG); // ends exactly where the button row begins (y=112)
  scrnSprite.setTextDatum(TL_DATUM);
  scrnSprite.setTextColor(COL_TEXT, COL_BG);
  char line[64];
  int y = 32;


  snprintf(line, sizeof(line), "Firmware version: %s", AUTO_VERSION);
  scrnSprite.drawString(line,10,y,1); y+= 13;

  snprintf(line, sizeof(line), "Firmware built: %s %s", __DATE__, __TIME__);
  scrnSprite.drawString(line, 10, y, 1); y += 13;
  
  unsigned long upSec = millis() / 1000;
  snprintf(line, sizeof(line), "Uptime: %luh %02lum %02lus", upSec / 3600, (upSec / 60) % 60, upSec % 60);
  scrnSprite.drawString(line, 10, y, 1); y += 13;

  snprintf(line, sizeof(line), "Free heap: %lu bytes", (unsigned long)ESP.getFreeHeap());
  scrnSprite.drawString(line, 10, y, 1); y += 13;
 if (WiFi.status() == WL_CONNECTED) {
    snprintf(line, sizeof(line), "WiFi: %s  IP: %s  RSSI: %d dBm",
             WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    snprintf(line, sizeof(line), "WiFi: not connected");
  }
  scrnSprite.drawString(line, 10, y, 1); y += 13;

  snprintf(line, sizeof(line), "Touch cal: X[%d,%d] Y[%d,%d]", tsMinX, tsMaxX, tsMinY, tsMaxY);
  scrnSprite.drawString(line, 10, y, 1); y += 13;

  scrnSprite.fillRect(0, y, SCREEN_W, 13, COL_BG);
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    snprintf(line, sizeof(line), "Raw touch: X=%d Y=%d (touch anywhere to test)", p.x, p.y);
  } else {
    snprintf(line, sizeof(line), "Raw touch: -- (touch anywhere to test)");
  }
  scrnSprite.drawString(line, 10, y, 1);
}

void drawService() {
  scrnSprite.fillSprite(COL_BG);
  scrnSprite.setTextDatum(TC_DATUM);
  scrnSprite.setTextColor(COL_ACCENT, COL_BG);
  scrnSprite.drawString("Service Menu", SCREEN_W / 2, 4, 2);
  resetArmed = false;
  drawServiceInfo();

  drawButton(btnSvcTestA, "Test Bell A");
  drawButton(btnSvcTestB, "Test Bell B");
  drawButton(btnSvcRestart, "Restart Device");
  drawButton(btnSvcReset, "Factory Reset", COL_BTN_OFF);
  //drawButton(btnSvcBack, "Back");
  drawIconButton(btnSvcBack, ICON_BACK);
}

void handleServiceTouch(int16_t x, int16_t y) {
  if (hit(btnSvcTestA, x, y)) { triggerBellA(); return; }
  if (hit(btnSvcTestB, x, y)) { triggerBellB(); return; }
  if (hit(btnSvcRestart, x, y)) { ESP.restart(); return; }
  if (hit(btnSvcReset, x, y)) {
    if (!resetArmed) {
      resetArmed = true;
      resetArmedAt = millis();
      drawButton(btnSvcReset, "Tap to CONFIRM", TFT_RED);
    } else {
      prefs.begin("belltimer", false);
      prefs.clear();
      prefs.end();
      ESP.restart();
    }
    return;
  }
  if (hit(btnSvcBack, x, y)) { currentScreen = SCR_HOME; drawHome(); return; }
}

// Long-press detector for the hidden entry gesture, plus the live info
// refresh while the service screen is open. Called every loop() iteration
// regardless of which screen is active.
void serviceMenuTick() {
  if (currentScreen == SCR_HOME) {
    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      int16_t mx = constrain(map(p.x, tsMinX, tsMaxX, 0, SCREEN_W), 0, SCREEN_W - 1);
      int16_t my = constrain(map(p.y, tsMinY, tsMaxY, 0, SCREEN_H), 0, SCREEN_H - 1);
      if (hit(svcHotspot, mx, my)) {
        if (svcHoldStart == 0) svcHoldStart = millis();
        else if (millis() - svcHoldStart >= SERVICE_HOLD_MS) {
          svcHoldStart = 0;
          touchWasDown = true; // swallow the release so it doesn't also fire a Home tap
          currentScreen = SCR_SERVICE;
          drawService();
        }
      } else {
        svcHoldStart = 0;
      }
    } else {
      svcHoldStart = 0;
    }
  } else if (currentScreen == SCR_SERVICE) {
    if (resetArmed && millis() - resetArmedAt > RESET_ARM_WINDOW_MS) {
      resetArmed = false;
      drawButton(btnSvcReset, "Factory Reset", COL_BTN_OFF);
    }
    static unsigned long lastInfoRefresh = 0;
    if (millis() - lastInfoRefresh > 400) { drawServiceInfo(); lastInfoRefresh = millis(); }
  }
}


// ---------------------------------------------------------------------------
// Screen: SCREENSAVER (idle display)
// ---------------------------------------------------------------------------
// A bouncing "DVD logo"-style box showing the current time, changing color
// each time it bounces off an edge. Purely cosmetic, but keeping the clock
// visible means the screen still does something useful while idle. Only
// ever entered from Home (see the idle check in loop()), and only ever
// exits back to Home - on any touch (see handleTouch() above) or
// automatically once a bell is due soon (see screensaverTick() below).
const int16_t SS_BOX_W = 140, SS_BOX_H = 50;
const int16_t SS_TOP_MARGIN = 0;
float ssX = 60, ssY = 60, ssVX = 1.6, ssVY = 1.3;
uint16_t ssColor = TFT_CYAN;
unsigned long lastSsFrame = 0;

uint16_t randomBounceColor() {
  uint16_t palette[] = { TFT_CYAN, TFT_YELLOW, TFT_MAGENTA, TFT_GREEN, TFT_ORANGE, COL_ACCENT };
  return palette[random(0, 6)];
}

// Backlight control - shares the idle timer above but is switched
// independently of the bouncing-clock visual. Fades smoothly over
// BACKLIGHT_FADE_MS rather than snapping instantly, using a time-based
// interpolation (not a fixed step count) so a small change and a large one
// both take the same duration rather than the large one taking longer.
// Non-blocking throughout - serviceBacklightFade() is called once per
// loop() iteration and only ever does simple arithmetic plus one ledcWrite.
#define BACKLIGHT_FADE_MS 300
uint8_t backlightDuty = 255;          // last value actually written to the PWM channel
uint8_t backlightFadeFrom = 255;
uint8_t backlightFadeTo = 255;
unsigned long backlightFadeStartMillis = 0;
bool backlightFading = false;

void applyBacklight(uint8_t duty) {
  backlightDuty = duty;
  ledcWrite(BACKLIGHT_PWM_CHANNEL, duty);
}

void fadeBacklightTo(uint8_t target) {
  if (!backlightFading && target == backlightDuty) return; // already there, nothing to do
  // Starting from backlightDuty (not backlightFadeFrom) means a fade that
  // gets reversed partway through - e.g. a touch arriving while the screen
  // is mid-fade-to-dim - continues smoothly from wherever it currently is,
  // rather than jumping back to where the previous fade started.
  backlightFadeFrom = backlightDuty;
  backlightFadeTo = target;
  backlightFadeStartMillis = millis();
  backlightFading = true;
}

void serviceBacklightFade() {
  if (!backlightFading) return;
  unsigned long elapsed = millis() - backlightFadeStartMillis;
  if (elapsed >= BACKLIGHT_FADE_MS) {
    applyBacklight(backlightFadeTo);
    backlightFading = false;
    return;
  }
  int delta = (int)backlightFadeTo - (int)backlightFadeFrom;
  applyBacklight((uint8_t)(backlightFadeFrom + (delta * (long)elapsed) / BACKLIGHT_FADE_MS));
}

void dimBacklight() {
  if (isDimmed) return;
  fadeBacklightTo((uint8_t)map(dimLevelPercent, 0, 100, 0, 255));
  isDimmed = true;
}

void undimBacklight() {
  if (!isDimmed) return;
  fadeBacklightTo(255);
  isDimmed = false;
}

void enterScreensaver() {
  scrnSprite.fillSprite(COL_BG);
  ssX = random(0, SCREEN_W - SS_BOX_W);
  ssY = random(SS_TOP_MARGIN, SCREEN_H - SS_BOX_H);
  ssVX = random(0, 2) ? 1.6 : -1.6;
  ssVY = random(0, 2) ? 1.3 : -1.3;
  ssColor = randomBounceColor();
  lastSsFrame = millis();
  currentScreen = SCR_SCREENSAVER;
}

void exitScreensaver() {
  undimBacklight();
  currentScreen = SCR_HOME;
  drawHome();
}

void screensaverTick() {
  if (currentScreen != SCR_SCREENSAVER) return;

  if (bellImminent()) { exitScreensaver(); return; } // wake early for an upcoming bell

  if (millis() - lastSsFrame < 40) return; // ~25fps, plenty smooth for this
  lastSsFrame = millis();

  scrnSprite.fillRect((int)ssX, (int)ssY, SS_BOX_W, SS_BOX_H, COL_BG); // erase old position


  ssX += ssVX;
  ssY += ssVY;
  bool bounced = false;
  if (ssX <= 0) { ssX = 0; ssVX = -ssVX; bounced = true; }
  if (ssX + SS_BOX_W >= SCREEN_W) { ssX = SCREEN_W - SS_BOX_W; ssVX = -ssVX; bounced = true; }
  if (ssY <= SS_TOP_MARGIN) { ssY = SS_TOP_MARGIN; ssVY = -ssVY; bounced = true; }
  if (ssY + SS_BOX_H >= SCREEN_H) { ssY = SCREEN_H - SS_BOX_H; ssVY = -ssVY; bounced = true; }
  if (bounced) ssColor = randomBounceColor();


  scrnSprite.fillRoundRect((int)ssX, (int)ssY, SS_BOX_W, SS_BOX_H, 8, ssColor);
  scrnSprite.drawRoundRect((int)ssX, (int)ssY, SS_BOX_W, SS_BOX_H, 8, TFT_WHITE);

  struct tm t;
  char hm[8] = "--:--";
  if (getNow(t)) snprintf(hm, sizeof(hm), "%02d:%02d", t.tm_hour, t.tm_min);
  
  scrnSprite.setTextDatum(MC_DATUM);
  scrnSprite.setTextColor(TFT_BLACK, ssColor);
  scrnSprite.drawString(hm, (int)ssX + SS_BOX_W / 2, (int)ssY + SS_BOX_H / 2, 4);

  //scrnSprite.pushSprite(0,0);
}

// ---------------------------------------------------------------------------
// Touch dispatch
// ---------------------------------------------------------------------------
void handleTouch() {
  int16_t x, y;
  bool down = getTouchPoint(x, y);
  if (down && !touchWasDown) {
    lastActivityMillis = millis(); // any tap counts as activity, on any screen
    undimBacklight();
    switch (currentScreen) {
      case SCR_HOME:     handleHomeTouch(x, y); break;
      case SCR_LIST:     handleListTouch(x, y); break;
      case SCR_EDIT:     handleEditTouch(x, y); break;
      case SCR_SETTINGS: handleSettingsTouch(x, y); break;
      case SCR_CALIB:    handleCalibTouch(); break;
      case SCR_SERVICE:  handleServiceTouch(x, y); break;
      case SCR_SCREENSAVER_CFG: handleScreensaverSettingsTouch(x, y); break;
      case SCR_SCREENSAVER: exitScreensaver(); break; // any tap wakes it
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
  randomSeed(esp_random());

  pinMode(BELL_A_PIN, OUTPUT); digitalWrite(BELL_A_PIN, LOW);
  pinMode(BELL_B_PIN, OUTPUT); digitalWrite(BELL_B_PIN, LOW);

  tft.init();

  // Take over the backlight pin with PWM (must happen after tft.init(),
  // which otherwise leaves it as a plain digitalWrite HIGH via
  // TFT_BACKLIGHT_ON in User_Setup.h) so it can be dimmed rather than only
  // switched fully on/off. COMPATIBILITY NOTE: this uses the ledcSetup +
  // ledcAttachPin form standard on arduino-esp32 core 2.x. Core 3.x
  // simplified this to a single ledcAttach(pin, freq, resolution) call and
  // writes duty via ledcWrite(pin, ...) directly instead of by channel - if
  // this fails to compile on core 3.x, swap to that form (same idea as the
  // watchdog note above).
  ledcSetup(BACKLIGHT_PWM_CHANNEL, BACKLIGHT_PWM_FREQ_HZ, BACKLIGHT_PWM_RES_BITS);
  ledcAttachPin(TFT_BL_PIN, BACKLIGHT_PWM_CHANNEL);
  applyBacklight(255); // full brightness at boot - no fade needed, nothing to fade from yet

#if ESP_IDF_VERSION_MAJOR == 5
  ledcAttach(LCD_BACK_LIGHT_PIN,LEDC_BASE_FREQ,LEDC_TIMER_12_BIT);
#else
//  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
//  ledcAttachPin(LCD_BACK_LIGHT_PIN,LEDC_CHANNEL_0);
#endif

  tft.setRotation(1); // landscape, 320x240
  tft.fillScreen(COL_BG);

  //ledcAnalogWrite(LEDC_CHANNEL_0, 128);


  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  int w = tft.width();
  int h = tft.height();

  scrnSprite.setColorDepth(8);
  scrnSprite.createSprite(w,h);
  scrnSprite.fillSprite(COL_BG);

  Serial.print("Width:"); Serial.println(w);
  Serial.print("Height:"); Serial.println(h);

  loadSchedules();

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("Connecting WiFi (or setup AP)...", SCREEN_W / 2, SCREEN_H / 2, 2);

  char ver[64];
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

  startWatchdog();

  polycomSetup();

  initWebServer();

  adminServer.on("/", HTTP_GET, handleAdminRoot);
  adminServer.on("/style.css", HTTP_GET, handleAdminCss);
  adminServer.on("/app.js", HTTP_GET, handleAdminJs);
  adminServer.on("/api/status", HTTP_GET, handleApiStatus);
  adminServer.on("/api/schedule", HTTP_GET, handleApiScheduleGet);
  adminServer.on("/api/timer", HTTP_POST, handleApiTimerPost);
  adminServer.on("/api/timer/delete", HTTP_POST, handleApiTimerDelete);
  adminServer.on("/api/schedule/enable", HTTP_POST, handleApiScheduleEnable);
  adminServer.on("/api/bell/test", HTTP_POST, handleApiBellTest);
  adminServer.on("/api/password", HTTP_POST, handleApiPassword);
  adminServer.begin();

  currentScreen = SCR_HOME;
  drawHome();
  lastActivityMillis = millis(); // start the idle clock from here, not from cold boot

  
}

void loop() {

  struct tm timeinfo;

  esp_task_wdt_reset();

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
  serviceMenuTick();
  adminServer.handleClient();
  screensaverTick();
  serviceBacklightFade();
  scrnSprite.pushSprite(0,0);


  if (currentScreen == SCR_HOME) {
    static unsigned long lastClock = 0;
    if (millis() - lastClock > 500) { updateHomeClock(false); lastClock = millis(); }

    bool idleNow = millis() - lastActivityMillis > (unsigned long)screensaverIdleMinutes * 60000UL;
    bool imminent = bellImminent();

    if (idleNow && !imminent) {
      if (dimEnabled) dimBacklight();
      if (screensaverEnabled) enterScreensaver();
    } else if (imminent && isDimmed) {
      undimBacklight();
    }
    
    //if (screensaverEnabled && !bellImminent() &&
    //    millis() - lastActivityMillis > (unsigned long)screensaverIdleMinutes * 60000UL) {
    //  enterScreensaver();
    //}
  }

  ElegantOTA.loop();

  delay(15);
}
