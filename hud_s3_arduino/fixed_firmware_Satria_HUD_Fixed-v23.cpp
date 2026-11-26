#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_Fingerprint.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <LovyanGFX.hpp>
#include <ChronosESP32.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <atomic>
#include <cstring>
#include <cctype>
#include <string>
#include "common_proto.h"

// ============================================================================
// KONFIGURASI PIN & HARDWARE (ESP32-S3 ST7789 170x320)
// ============================================================================
constexpr int PIN_LCD_RST  = 5;
constexpr int PIN_LCD_CS   = 6;
constexpr int PIN_LCD_DC   = 7;
constexpr int PIN_LCD_SCLK = 8;
constexpr int PIN_LCD_MOSI = 9;
constexpr int PIN_LCD_BL   = 38;

constexpr int PIN_BTN_MAIN   = 1;
constexpr int PIN_BTN_EMRG   = 2;
constexpr int PIN_BTN_ENROLL = 3;

constexpr int PIN_RELAY      = 10;
constexpr int PIN_RELAY_SEAT = 11;
constexpr int PIN_LED_NOTIF  = 12;
constexpr int PIN_BUZZER     = 13;

constexpr int PIN_I2C_SDA  = 16;
constexpr int PIN_I2C_SCL  = 21;

constexpr int PIN_R503_RX  = 17;
constexpr int PIN_R503_TX  = 18;

constexpr uint8_t  LCD_BL_PWM_CH     = 0;
constexpr uint32_t LCD_BL_PWM_FREQ   = 5000;
constexpr uint8_t  LCD_BL_PWM_RES    = 8;
constexpr uint8_t  LCD_BL_DUTY_DAY   = 240;
constexpr uint8_t  LCD_BL_DUTY_NIGHT = 115;
constexpr uint16_t COL_FRAME         = 0x2104;

// ============================================================================
// KONFIGURASI PROTOKOL, SMARTWATCH & AMBANG BATAS
// ============================================================================
constexpr uint8_t TCI_WIFI_CHANNEL = 1;
static const uint8_t HUD_S3_CUSTOM_MAC[6] = {0x02, 0x53, 0x46, 0x55, 0x48, 0x55};
static const uint8_t DEFAULT_C3_MAC[6]     = {0x02, 0x53, 0x46, 0x55, 0x43, 0x33};

static constexpr char TARGET_WATCH_NAME[]   = "Galaxy Watch4 (3M7H)";
static constexpr int  WATCH_UNLOCK_RSSI_MIN = -75;
static constexpr unsigned long WATCH_SCAN_INTERVAL_MS = 1500;
static constexpr unsigned long WATCH_SCAN_WINDOW_MS   = 80;

constexpr bool STRICT_C3_PEER_LOCK     = true;
constexpr bool ALLOW_INITIAL_C3_LEARN  = true;
constexpr bool RESET_EMG_ON_BOOT       = true;

constexpr uint8_t       FINGER_MAX_FAIL_STREAK = 10;
constexpr unsigned long FINGER_FAIL_COOLDOWN_MS = 5000;

constexpr unsigned long NAV_POLL_MS   = 150;
constexpr unsigned long NOTIF_POLL_MS = 150;

constexpr char BLE_DEVICE_NAME[] = "Satria-HUD";
constexpr char NVS_NAMESPACE[]   = "satria_hud";

constexpr BaseType_t CORE_COMM_UI  = 0;
constexpr BaseType_t CORE_SECURITY = 1;

constexpr uint16_t MAX_RPM_SCALE     = 14000;
constexpr uint16_t IDLE_MAX_RPM_MODE = 400;
constexpr uint16_t RPM_WARN_YELLOW   = 7500;
constexpr uint16_t RPM_WARN_RED      = 10500;

constexpr uint16_t BATT_MIN_WARN_10 = 115;
constexpr uint16_t BATT_MAX_WARN_10 = 155;
constexpr int16_t  ENG_TEMP_WARN_10 = 950;

constexpr unsigned long DEBOUNCE_MS        = 50;
constexpr unsigned long LONG_PRESS_MS      = 3000;
constexpr unsigned long SEAT_MAX_HOLD_MS   = 8000;
constexpr unsigned long EMERGENCY_HOLD_MS  = 3500;
constexpr unsigned long ENROLL_BTN_HOLD_MS = 3000;
constexpr unsigned long DOUBLE_TAP_MS      = 300;
constexpr unsigned long TCI_TIMEOUT_MS     = 1500;
constexpr unsigned long NAV_TIMEOUT_MS     = 10000;
constexpr unsigned long NOTIF_POPUP_MS     = 5000;
constexpr unsigned long MODE_RETRY_INT_MS  = 450;
constexpr unsigned long ENROLL_STEP_TMO_MS = 6000;
constexpr uint16_t      FINGER_MAX_SLOTS   = 127;

constexpr int      ADV_HIST_LEN        = 160;
constexpr uint32_t WDT_TIMEOUT_SEC     = 5;
constexpr uint32_t I2C_CLOCK_HZ        = 100000;
constexpr uint8_t  MODE_SEND_MAX_RETRY = 5;

constexpr unsigned long RTC_READ_MS      = 500;
constexpr unsigned long FINGER_POLL_MS   = 120;
constexpr unsigned long TICKER_STEP_MS   = 95;
constexpr unsigned long CURVE_REDRAW_MS  = 60;

constexpr uint16_t FAULT_NONE      = 0;
constexpr uint16_t FAULT_TPS_OUT   = 1u << 0;
constexpr uint16_t FAULT_NTC_OUT   = 1u << 1;
constexpr uint16_t FAULT_BATT_LOW  = 1u << 2;
constexpr uint16_t FAULT_BATT_HIGH = 1u << 3;
constexpr uint16_t FAULT_NVS_ERR   = 1u << 4;
constexpr uint16_t FAULT_ADC_ERR   = 1u << 5;

constexpr uint8_t STATE_STOPPED   = 0;
constexpr uint8_t STATE_CRANKING  = 1;
constexpr uint8_t STATE_RUNNING   = 2;
constexpr uint8_t STATE_LIMP_HOME = 3;

// ============================================================================
// DRIVER LOVYANGFX (ST7789 170x320)
// ============================================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = PIN_LCD_SCLK;
      cfg.pin_mosi    = PIN_LCD_MOSI;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = PIN_LCD_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = PIN_LCD_CS;
      cfg.pin_rst          = PIN_LCD_RST;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 170;
      cfg.panel_height     = 320;
      cfg.offset_x         = 35;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

static LGFX lcd;
static RTC_DS3231 rtc;
static Preferences prefs;
static std::atomic<bool>    g_rtcOk{false};
static std::atomic<bool>    g_rtcLostPower{false};
static std::atomic<uint8_t> g_rtcHour{12};

static HardwareSerial r503Serial(2);
static Adafruit_Fingerprint finger(&r503Serial);
static std::atomic<bool>          g_r503Ok{false};
static std::atomic<uint8_t>       g_lastR503Err{0};
static std::atomic<uint8_t>       g_fingerFailStreak{0};
static std::atomic<bool>          g_fingerCooldownActive{false};
static std::atomic<unsigned long> g_fingerCooldownStartMs{0};

static ChronosESP32 watch(BLE_DEVICE_NAME);

static std::atomic<bool>          g_watchOneShotMatched{false};
static std::atomic<bool>          g_watchScanFinished{false};
static std::atomic<bool>          g_watchScanInProgress{false};
static std::atomic<unsigned long> g_watchScanStartMs{0};
static std::atomic<int>           g_watchMatchedRssi{-127};
static NimBLEScan* g_pWatchScan = nullptr;

// ============================================================================
// STRUKTUR TELEMETRI & PERINTAH
// ============================================================================
#ifndef COMMON_PROTO_STRUCTS_DEFINED
#define COMMON_PROTO_STRUCTS_DEFINED
struct __attribute__((packed)) TelemetryData {
  uint16_t header;
  uint16_t rpm;
  uint8_t  tps;
  int16_t  degree10;
  int16_t  temp10;
  uint16_t battVolt10;
  uint8_t  mode;
  uint8_t  fuel;
  uint8_t  state;
  uint8_t  ackStatus;
  uint16_t faultCode;
  uint16_t crc16;
};

struct __attribute__((packed)) CommandData {
  uint16_t header;
  uint8_t  requestedMode;
  uint16_t crc16;
};
#endif

static_assert(sizeof(TelemetryData) == 19, "Ukuran TelemetryData harus 19 byte");
static_assert(sizeof(CommandData) == 5, "Ukuran CommandData tidak sesuai");

struct EspNowRawFrame {
  uint8_t srcMac[6];
  uint8_t payload[sizeof(TelemetryData)];
  int     len;
  unsigned long rxMs;
};

static uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static uint16_t getRedlineRPM(uint8_t mode) {
  switch (mode) {
    case 1:  return 11000;
    case 2:  return 12000;
    case 3:  return 11500;
    default: return 9000;
  }
}

// ============================================================================
// NAVIGASI & NOTIFIKASI CHRONOS
// ============================================================================
enum NavTurnType : uint8_t {
  NAV_TURN_NONE = 0,
  NAV_TURN_STRAIGHT,
  NAV_TURN_LEFT,
  NAV_TURN_RIGHT,
  NAV_TURN_SLIGHT_LEFT,
  NAV_TURN_SLIGHT_RIGHT,
  NAV_TURN_U_TURN,
  NAV_TURN_ROUNDABOUT,
  NAV_TURN_ARRIVE
};

struct NavTelemetryData {
  bool          bleConnected;
  bool          navActive;
  NavTurnType   turn;
  char          turnLabel[14];
  char          distance[16];
  char          speedInfo[16];
  char          roadName[36];
  char          etaInfo[24];
  char          durationInfo[18];
  unsigned long lastRxMs;
  uint32_t      seq;
};

static portMUX_TYPE g_navMux = portMUX_INITIALIZER_UNLOCKED;
static NavTelemetryData g_navData = {
  false, false, NAV_TURN_NONE, "STANDBY", "--", "-- KM/J",
  "Buka GMaps & Chronos", "--", "--", 0, 0
};

struct NotifPopupData {
  bool          active;
  char          app[20];
  char          title[28];
  char          message[96];
  unsigned long startMs;
  unsigned long durationMs;
  uint32_t      seq;
};

static portMUX_TYPE g_notifMux = portMUX_INITIALIZER_UNLOCKED;
static NotifPopupData g_notifPopup = {false, "", "", "", 0, NOTIF_POPUP_MS, 0};

// ============================================================================
// STATE SISTEM & RTOS
// ============================================================================
enum BootState : uint8_t {
  BOOT_WAIT_FINGERPRINT = 0,
  BOOT_UNLOCKED_ACTIVE
};

enum PageId : uint8_t {
  PAGE_HOME = 1,
  PAGE_NAV  = 2,
  PAGE_TCI  = 3
};

enum Core0CmdType : uint8_t {
  CMD_PAGE_CHANGED = 1,
  CMD_SUBPAGE_TOGGLE,
  CMD_TCI_NEXT_MODE,
  CMD_SYSTEM_LOCKED,
  CMD_SAVE_EMG_STATE,
  CMD_RESET_C3_MAC
};

struct Core0Command {
  Core0CmdType type;
  PageId       prevPage;
  PageId       nextPage;
  bool         flagVal;
};

static BootState     g_bootState       = BOOT_WAIT_FINGERPRINT;
static PageId        g_currentPage     = PAGE_HOME;
static bool          g_systemUnlocked  = false;
static bool          g_emergencyByPass = false;
static bool          g_needFullClear   = true;
static bool          g_seatRelayActive = false;
static unsigned long g_seatOpenStartMs = 0;

static std::atomic<bool> g_tciEspNowReady{false};
static std::atomic<bool> g_tciRxEnabled{false};
static std::atomic<bool> g_allowChronosNotif{true};
static std::atomic<bool> g_watchStopPending{false};

static SemaphoreHandle_t g_i2cMutex  = nullptr;
static SemaphoreHandle_t g_nvsMutex  = nullptr;
static SemaphoreHandle_t g_r503Mutex = nullptr;

static portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
static char g_fingerStatus[32] = "R503 + GW4 Ready";
static char g_timeStr[12]      = "00:00:00";
static char g_dateStr[16]      = "01-01-2026";
static float g_chipTempC       = 0.0f;
static uint32_t g_rtcSeq       = 0;

static portMUX_TYPE g_tciMux = portMUX_INITIALIZER_UNLOCKED;
static TelemetryData g_tciPkt = {0xAA55, 0, 0, 100, 300, 126, 0, 0, 0, 0, 0, 0};
static volatile unsigned long g_lastTciRxMs = 0;
static volatile uint32_t g_tciRxSeq       = 0;
static uint8_t g_advHistory[ADV_HIST_LEN] = {0};
static int     g_advHistIdx = 0;

static uint8_t          g_tciSubPage        = 0;
static unsigned long    g_popupStartMs      = 0;
static unsigned long    g_popupDurationMs   = 0;
static uint32_t         g_popupSeq          = 0;
static char             g_popupLine1[20]    = "";
static char             g_popupLine2[20]    = "";
static uint16_t         g_popupBgColor      = TFT_WHITE;

static bool             g_pendingModeChange = false;
static uint8_t          g_desiredMode       = 0;
static uint8_t          g_modeRetryCount    = 0;
static unsigned long    g_lastModeSendMs    = 0;
static uint32_t         g_modeReqStartSeq   = 0;
static volatile bool    g_lastEspSendOk     = false;
static volatile bool    g_espSendCbSeen     = false;

static uint8_t          activeC3PeerMac[6]  = {0x02, 0x53, 0x46, 0x55, 0x43, 0x33};
static volatile bool    g_c3MacLocked       = false;
static volatile bool    g_newPeerPending    = false;
static uint8_t          g_pendingPeerMac[6] = {0};

static QueueHandle_t g_core0CmdQueue = nullptr;
static QueueHandle_t g_espNowRxQueue = nullptr;

// ============================================================================
// UTILITAS & LOCK
// ============================================================================
static inline void copyStr(char *dst, size_t dstSize, const char *src) {
  if (!dst || dstSize == 0) return;
  strncpy(dst, src ? src : "", dstSize - 1);
  dst[dstSize - 1] = '\0';
}
template <size_t N>
static inline void copyStr(char (&dst)[N], const char *src) { copyStr(dst, N, src); }

class CritSection {
public:
  explicit CritSection(portMUX_TYPE &m) : _m(m) { portENTER_CRITICAL(&_m); }
  ~CritSection() { portEXIT_CRITICAL(&_m); }
  CritSection(const CritSection &) = delete;
  CritSection &operator=(const CritSection &) = delete;
private:
  portMUX_TYPE &_m;
};

class ScopedLock {
public:
  ScopedLock(SemaphoreHandle_t m, uint32_t timeoutMs, bool passIfNull = false)
    : _m(m),
      _held(m && xSemaphoreTake(m, pdMS_TO_TICKS(timeoutMs)) == pdTRUE),
      _ok(_held || (!m && passIfNull)) {}
  ~ScopedLock() { release(); }
  void release() { if (_held) { xSemaphoreGive(_m); _held = false; } }
  explicit operator bool() const { return _ok; }
  ScopedLock(const ScopedLock &) = delete;
  ScopedLock &operator=(const ScopedLock &) = delete;
private:
  SemaphoreHandle_t _m;
  bool _held;
  bool _ok;
};

struct GfxBatch {
  GfxBatch()  { lcd.startWrite(); }
  ~GfxBatch() { lcd.endWrite(); }
  GfxBatch(const GfxBatch &) = delete;
  GfxBatch &operator=(const GfxBatch &) = delete;
};

static inline bool isTciOnline(unsigned long lastRxMs, unsigned long nowMs) {
  return (lastRxMs > 0) && ((nowMs - lastRxMs) < TCI_TIMEOUT_MS);
}

static void postCore0(Core0CmdType type, PageId prev = PAGE_HOME, PageId next = PAGE_HOME, bool flag = false) {
  if (!g_core0CmdQueue) return;
  Core0Command cmd = {type, prev, next, flag};
  xQueueSend(g_core0CmdQueue, &cmd, 0);
}

static void setFingerStatus(const char *text, bool needFullClear = false) {
  CritSection cs(g_stateMux);
  copyStr(g_fingerStatus, text);
  if (needFullClear) g_needFullClear = true;
}

static void triggerPopup(const char* l1, const char* l2, uint16_t bgColor, uint16_t durationMs = 1500);
static void syncTelemetryGate(PageId page);
static void initTciEspNowOnce();
static void requestStopWatchScanOnce();
static void serviceStopWatchScanOnCore0();

// ============================================================================
// PWM BACKLIGHT & NVS
// ============================================================================
static inline bool isTimerActive(unsigned long startMs, unsigned long durationMs, unsigned long nowMs) {
  return (durationMs > 0) && ((nowMs - startMs) < durationMs);
}

static void setLcdBacklightDuty(uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LCD_BL, duty);
#else
  ledcWrite(LCD_BL_PWM_CH, duty);
#endif
}

static void initLcdBacklightPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_LCD_BL, LCD_BL_PWM_FREQ, LCD_BL_PWM_RES);
#else
  ledcSetup(LCD_BL_PWM_CH, LCD_BL_PWM_FREQ, LCD_BL_PWM_RES);
  ledcAttachPin(PIN_LCD_BL, LCD_BL_PWM_CH);
#endif
  setLcdBacklightDuty(LCD_BL_DUTY_DAY);
}

static void updateAdaptiveBacklight() {
  static uint8_t lastDuty = 255;
  uint8_t targetDuty = LCD_BL_DUTY_DAY;
  if (g_rtcOk.load(std::memory_order_acquire)) {
    uint8_t hr = g_rtcHour.load(std::memory_order_acquire);
    if (hr >= 18 || hr < 6) targetDuty = LCD_BL_DUTY_NIGHT;
  }
  if (targetDuty != lastDuty) {
    lastDuty = targetDuty;
    setLcdBacklightDuty(targetDuty);
  }
}

template <typename Fn>
static void nvsWrite(Fn &&fn) {
  ScopedLock lock(g_nvsMutex, 100);
  if (!lock) return;
  if (prefs.begin(NVS_NAMESPACE, false)) {
    fn(prefs);
    prefs.end();
  }
}

static void saveEmergencyStateToNvs(bool emgState) {
  nvsWrite([&](Preferences &p) { p.putBool("emg_bypass", emgState); });
}

static void saveC3MacToNvs(const uint8_t mac[6]) {
  nvsWrite([&](Preferences &p) { p.putBytes("c3_mac", mac, 6); });
}

static void clearC3MacFromNvs() {
  nvsWrite([](Preferences &p) { p.remove("c3_mac"); });
}

static void loadPersistentSettings() {
  bool savedEmg = false;
  {
    ScopedLock lock(g_nvsMutex, 100);
    if (!lock) return;

    bool hasMac = false;
    uint8_t savedMac[6] = {0};
    if (prefs.begin(NVS_NAMESPACE, true)) {
      savedEmg = prefs.getBool("emg_bypass", false);
      hasMac = (prefs.getBytesLength("c3_mac") == 6 && prefs.getBytes("c3_mac", savedMac, 6) == 6);
      prefs.end();
    }

    const bool activeEmgOnBoot = RESET_EMG_ON_BOOT ? false : savedEmg;

    CritSection cs(g_stateMux);
    g_emergencyByPass = activeEmgOnBoot;
    if (activeEmgOnBoot)  copyStr(g_fingerStatus, "EMERGENCY ON");
    else if (savedEmg)    copyStr(g_fingerStatus, "LOCKED (EMG RST)");
    if (hasMac) {
      memcpy(activeC3PeerMac, savedMac, 6);
      g_c3MacLocked = true;
    }
  }

  if (RESET_EMG_ON_BOOT && savedEmg) {
    saveEmergencyStateToNvs(false);
  }
}

static void sanitizeAsciiInPlace(char *s, bool replaceNewlineWithSpace = true) {
  if (!s) return;
  size_t w = 0;
  for (size_t r = 0; s[r] != '\0'; r++) {
    unsigned char ch = static_cast<unsigned char>(s[r]);
    if (ch == '\r') continue;
    if (ch == '\n') s[w++] = replaceNewlineWithSpace ? ' ' : '\n';
    else if (ch >= 0x20 && ch <= 0x7E) s[w++] = static_cast<char>(ch);
    else s[w++] = '?';
  }
  s[w] = '\0';
}

static void trimAsciiInPlace(char *s) {
  if (!s) return;
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\r' || s[len - 1] == '\n' || s[len - 1] == '\t')) {
    s[--len] = '\0';
  }
  size_t start = 0;
  while (s[start] == ' ' || s[start] == '\r' || s[start] == '\n' || s[start] == '\t') start++;
  if (start > 0) memmove(s, s + start, len - start + 1);
}

template <size_t N>
static void copyCleanAscii(char (&dst)[N], const char *src, bool newlineToSpace = true) {
  copyStr(dst, N, src);
  sanitizeAsciiInPlace(dst, newlineToSpace);
  trimAsciiInPlace(dst);
}

static bool containsIgnoreCase(const char *haystack, const char *needle) {
  if (!haystack || !needle || needle[0] == '\0') return false;
  size_t nLen = strlen(needle);
  for (size_t i = 0; haystack[i] != '\0'; i++) {
    size_t j = 0;
    while (j < nLen && haystack[i + j] != '\0' &&
           tolower(static_cast<unsigned char>(haystack[i + j])) ==
           tolower(static_cast<unsigned char>(needle[j]))) {
      j++;
    }
    if (j == nLen) return true;
  }
  return false;
}

static uint8_t getTciSubPageAtomic() {
  CritSection cs(g_stateMux);
  return g_tciSubPage;
}

static void setTciSubPageAtomic(uint8_t sp, bool fullClear) {
  CritSection cs(g_stateMux);
  g_tciSubPage = sp;
  if (fullClear) g_needFullClear = true;
}

static inline void applyRelayOutput(unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  bool relayOn;
  bool seatOn;
  {
    CritSection cs(g_stateMux);
    relayOn = (g_systemUnlocked || g_emergencyByPass);
    if (g_seatRelayActive && ((nowMs - g_seatOpenStartMs) >= SEAT_MAX_HOLD_MS)) {
      g_seatRelayActive = false;
    }
    seatOn = g_seatRelayActive;
  }
  digitalWrite(PIN_RELAY, relayOn ? HIGH : LOW);
  digitalWrite(PIN_RELAY_SEAT, seatOn ? HIGH : LOW);
  if (relayOn) requestStopWatchScanOnce();
}

static void setSeatRelayHold(bool active, unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  bool changed = false;
  {
    CritSection cs(g_stateMux);
    if (g_seatRelayActive != active) {
      g_seatRelayActive = active;
      changed = true;
      if (active) g_seatOpenStartMs = nowMs;
    }
  }
  applyRelayOutput(nowMs);
  if (active && changed) {
    triggerPopup("JOK TERBUKA", "LEPAS TOMBOL OFF", TFT_CYAN, 2000);
  }
}

static void setSystemStateAtomic(BootState bState, PageId page, bool unlocked, bool fullClear) {
  CritSection cs(g_stateMux);
  g_bootState      = bState;
  g_currentPage    = page;
  g_systemUnlocked = unlocked;
  if (fullClear) g_needFullClear = true;
}

static PageId getCurrentPageAtomic() {
  CritSection cs(g_stateMux);
  return g_currentPage;
}

static bool isSystemUnlockedAtomic() {
  CritSection cs(g_stateMux);
  return g_systemUnlocked;
}

static bool isSeatRelayActiveAtomic() {
  CritSection cs(g_stateMux);
  return g_seatRelayActive;
}

static bool isRelayAuthorizedAtomic() {
  CritSection cs(g_stateMux);
  return (g_systemUnlocked || g_emergencyByPass);
}

static bool isUnlockedAndOnTciPage() {
  if (!g_tciRxEnabled.load(std::memory_order_acquire)) return false;
  CritSection cs(g_stateMux);
  return ((g_systemUnlocked || g_emergencyByPass) && g_currentPage == PAGE_TCI);
}

static void triggerPopup(const char* l1, const char* l2, uint16_t bgColor, uint16_t durationMs) {
  const unsigned long startMs = millis();
  CritSection cs(g_stateMux);
  copyStr(g_popupLine1, l1);
  copyStr(g_popupLine2, l2);
  g_popupBgColor    = bgColor;
  g_popupStartMs    = startMs;
  g_popupDurationMs = durationMs;
  g_popupSeq++;
}

// ============================================================================
// ONE-SHOT SMARTWATCH BLE IMMOBILIZER (NIMBLE 2.x)
// ============================================================================
class WatchOneShotCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (!advertisedDevice) return;
    if (g_watchOneShotMatched.load(std::memory_order_acquire) ||
        g_watchScanFinished.load(std::memory_order_acquire)) {
      return;
    }
    if (!advertisedDevice->haveName()) return;

    const std::string devName = advertisedDevice->getName();
    if (strcmp(devName.c_str(), TARGET_WATCH_NAME) == 0) {
      const int rssi = advertisedDevice->getRSSI();
      if (rssi >= WATCH_UNLOCK_RSSI_MIN) {
        g_watchMatchedRssi.store(rssi, std::memory_order_release);
        g_watchOneShotMatched.store(true, std::memory_order_release);
      }
    }
  }
};

static WatchOneShotCallbacks g_watchScanCb;

static void requestStopWatchScanOnce() {
  if (!g_watchScanFinished.exchange(true, std::memory_order_acq_rel)) {
    g_watchStopPending.store(true, std::memory_order_release);
  }
}

static void serviceStopWatchScanOnCore0() {
  if (!g_watchStopPending.exchange(false, std::memory_order_acq_rel)) return;
  g_watchScanInProgress.store(false, std::memory_order_release);
  if (g_pWatchScan != nullptr) {
    g_pWatchScan->stop();
    g_pWatchScan->clearResults();
  }
}

static void pollWatchOneShotScan(unsigned long nowMs) {
  static unsigned long lastScanStartMs = 0;

  serviceStopWatchScanOnCore0();

  if (g_watchScanFinished.load(std::memory_order_acquire) || isRelayAuthorizedAtomic()) {
    requestStopWatchScanOnce();
    serviceStopWatchScanOnCore0();
    return;
  }

  if (g_pWatchScan == nullptr) {
    g_pWatchScan = NimBLEDevice::getScan();
    if (g_pWatchScan) {
      g_pWatchScan->setScanCallbacks(&g_watchScanCb, false);
      g_pWatchScan->setActiveScan(true);
      g_pWatchScan->setInterval(80);
      g_pWatchScan->setWindow(40);
    }
  }
  if (g_pWatchScan == nullptr) return;

  if (g_watchScanInProgress.load(std::memory_order_acquire)) {
    if ((nowMs - g_watchScanStartMs.load(std::memory_order_acquire)) >= WATCH_SCAN_WINDOW_MS ||
        g_watchOneShotMatched.load(std::memory_order_acquire)) {
      g_pWatchScan->stop();
      g_pWatchScan->clearResults();
      g_watchScanInProgress.store(false, std::memory_order_release);
    }
    return;
  }

  if ((nowMs - lastScanStartMs) < WATCH_SCAN_INTERVAL_MS) return;
  lastScanStartMs = nowMs;
  g_watchScanStartMs.store(nowMs, std::memory_order_release);
  g_watchScanInProgress.store(true, std::memory_order_release);
  g_pWatchScan->start(0, false, true);
}

// ============================================================================
// I2C & RTC DS3231
// ============================================================================
static void beginI2c() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(50);
}

static void recoverI2cBus() {
  ScopedLock lock(g_i2cMutex, 200);
  if (!lock) return;
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
  }
  beginI2c();
}

static void updateRtcCache() {
  if (!g_rtcOk.load(std::memory_order_acquire)) return;

  ::DateTime now;
  {
    ScopedLock lock(g_i2cMutex, 150);
    if (!lock) return;
    now = rtc.now();
  }

  if (now.year() < 2020 || now.year() > 2099 || now.hour() > 23 || now.minute() > 59 || now.second() > 59) {
    recoverI2cBus();
    return;
  }

  g_rtcHour.store(now.hour(), std::memory_order_release);

  char tBuf[12];
  char dBuf[16];
  snprintf(tBuf, sizeof(tBuf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  snprintf(dBuf, sizeof(dBuf), "%02d-%02d-%04d", now.day(), now.month(), now.year());

  float chipC = temperatureRead();
  if (isnan(chipC) || chipC < -20.0f || chipC > 125.0f) chipC = 0.0f;

  CritSection cs(g_stateMux);
  memcpy(g_timeStr, tBuf, sizeof(g_timeStr));
  memcpy(g_dateStr, dBuf, sizeof(dBuf));
  g_chipTempC = chipC;
  g_rtcSeq++;
}

// ============================================================================
// INDIKATOR LED & BUZZER (NON-BLOCKING)
// ============================================================================
enum BuzzerPattern : uint8_t {
  BUZZ_NONE = 0,
  BUZZ_UNLOCK,
  BUZZ_ALERT,
  BUZZ_ENROLL_STEP,
  BUZZ_ENROLL_OK,
  BUZZ_NOTIF
};

struct NotifHardwareState {
  uint8_t       ledMode;
  unsigned long ledStartMs;
  unsigned long ledDurationMs;
  unsigned long ledLastToggleMs;
  bool          ledPinState;

  BuzzerPattern buzzPat;
  uint8_t       buzzStep;
  unsigned long buzzStepStartMs;
  bool          buzzActiveTone;
};

static portMUX_TYPE g_hwNotifMux = portMUX_INITIALIZER_UNLOCKED;
static NotifHardwareState g_hwNotif = {0, 0, 0, 0, false, BUZZ_NONE, 0, 0, false};

static void triggerLedAndBuzzer(uint8_t ledMode, uint16_t ledDurationMs, BuzzerPattern pat, unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  portENTER_CRITICAL(&g_hwNotifMux);
  if (ledMode > 0) {
    g_hwNotif.ledMode         = ledMode;
    g_hwNotif.ledStartMs      = nowMs;
    g_hwNotif.ledDurationMs   = ledDurationMs;
    g_hwNotif.ledLastToggleMs = nowMs;
    g_hwNotif.ledPinState     = true;
  }
  if (pat != BUZZ_NONE) {
    g_hwNotif.buzzPat         = pat;
    g_hwNotif.buzzStep        = 0;
    g_hwNotif.buzzStepStartMs = nowMs;
    g_hwNotif.buzzActiveTone  = true;
  }
  portEXIT_CRITICAL(&g_hwNotifMux);
}

struct BuzzStep { bool on; uint16_t ms; };
struct BuzzSeq  { const BuzzStep *steps; uint8_t len; };

static constexpr BuzzStep BUZZ_SEQ_UNLOCK[]      = {{true, 90}, {false, 60}, {true, 140}};
static constexpr BuzzStep BUZZ_SEQ_ALERT[]       = {{true, 120}, {false, 90}, {true, 220}};
static constexpr BuzzStep BUZZ_SEQ_ENROLL_STEP[] = {{true, 80}};
static constexpr BuzzStep BUZZ_SEQ_ENROLL_OK[]   = {{true, 80}, {false, 70}, {true, 80}, {false, 70}, {true, 180}};
static constexpr BuzzStep BUZZ_SEQ_NOTIF[]       = {{true, 60}};

template <size_t N>
static constexpr BuzzSeq makeSeq(const BuzzStep (&a)[N]) { return {a, static_cast<uint8_t>(N)}; }

static constexpr BuzzSeq BUZZ_SEQS[] = {
  {nullptr, 0},
  makeSeq(BUZZ_SEQ_UNLOCK),
  makeSeq(BUZZ_SEQ_ALERT),
  makeSeq(BUZZ_SEQ_ENROLL_STEP),
  makeSeq(BUZZ_SEQ_ENROLL_OK),
  makeSeq(BUZZ_SEQ_NOTIF)
};
static_assert(sizeof(BUZZ_SEQS) / sizeof(BUZZ_SEQS[0]) == BUZZ_NOTIF + 1, "BUZZ_SEQS mismatch");

static void serviceLedAndBuzzer(bool systemUnlockedOrEmg, unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  bool outLed  = false;
  bool outBuzz = false;

  {
    CritSection cs(g_hwNotifMux);
    NotifHardwareState &h = g_hwNotif;

    if (h.ledMode != 0 && (nowMs - h.ledStartMs) >= h.ledDurationMs) {
      h.ledMode     = 0;
      h.ledPinState = false;
    }
    if (h.ledMode == 1) {
      outLed = true;
    } else if (h.ledMode == 2 || h.ledMode == 3) {
      const unsigned long halfPeriodMs = (h.ledMode == 2) ? 120 : 450;
      if ((nowMs - h.ledLastToggleMs) >= halfPeriodMs) {
        h.ledLastToggleMs = nowMs;
        h.ledPinState     = !h.ledPinState;
      }
      outLed = h.ledPinState;
    } else {
      outLed = systemUnlockedOrEmg;
    }

    if (h.buzzPat != BUZZ_NONE) {
      const BuzzSeq  &seq  = BUZZ_SEQS[h.buzzPat];
      const BuzzStep &step = seq.steps[h.buzzStep];
      outBuzz = step.on;
      if ((nowMs - h.buzzStepStartMs) >= step.ms) {
        if (h.buzzStep + 1 < seq.len) {
          h.buzzStep++;
          h.buzzStepStartMs = nowMs;
        } else {
          h.buzzPat  = BUZZ_NONE;
          h.buzzStep = 0;
          outBuzz    = false;
        }
      }
    }
  }

  digitalWrite(PIN_LED_NOTIF, outLed ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, outBuzz ? HIGH : LOW);
}

// ============================================================================
// SENSOR SIDIK JARI R503 & ENROLL TOMBOL FISIK (CORE 1)
// ============================================================================
static void applyR503DefaultConfig() {
  finger.setSecurityLevel(2);
  finger.LEDcontrol(FINGERPRINT_LED_BREATHING, 100, FINGERPRINT_LED_BLUE, 0);
}

static int scanFingerprintOnce(uint16_t &outId, int &outConfidence) {
  ScopedLock lock(g_r503Mutex, 100);
  if (!lock) return -1;

  if (!g_r503Ok.load(std::memory_order_acquire)) {
    const bool ok = finger.verifyPassword();
    g_r503Ok.store(ok, std::memory_order_release);
    if (!ok) {
      g_lastR503Err.store(FINGERPRINT_PACKETRECIEVEERR, std::memory_order_release);
      return -1;
    }
    applyR503DefaultConfig();
    g_lastR503Err.store(0, std::memory_order_release);
  }

  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
    if (p != FINGERPRINT_NOFINGER) {
      g_lastR503Err.store(p, std::memory_order_release);
      if (p == FINGERPRINT_PACKETRECIEVEERR) g_r503Ok.store(false, std::memory_order_release);
    }
    return -1;
  }

  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) {
    g_lastR503Err.store(p, std::memory_order_release);
    return -1;
  }

  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    g_lastR503Err.store(0, std::memory_order_release);
    outId = finger.fingerID;
    outConfidence = static_cast<int>(finger.confidence);
    finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 10, FINGERPRINT_LED_PURPLE, 2);
    return 0;
  }

  g_lastR503Err.store(p, std::memory_order_release);
  finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 10, FINGERPRINT_LED_RED, 2);
  return -2;
}

enum EnrollFsmState : uint8_t {
  ENROLL_IDLE = 0,
  ENROLL_FIND_SLOT,
  ENROLL_WAIT_FINGER_1,
  ENROLL_WAIT_LIFT,
  ENROLL_WAIT_FINGER_2
};

struct EnrollContext {
  EnrollFsmState state;
  uint16_t       scanSlotCursor;
  uint16_t       targetSlot;
  unsigned long  stepStartMs;
  unsigned long  lastSlotProbeMs;
};

static EnrollContext g_enrollCtx = {ENROLL_IDLE, 1, 0, 0, 0};
static constexpr uint16_t ENROLL_SLOTS_PER_TICK = 8;
static constexpr unsigned long ENROLL_SLOT_PROBE_GAP_MS = 20;

static void abortEnrollSession(const char *reasonMsg, unsigned long nowMs) {
  g_enrollCtx.state = ENROLL_IDLE;
  g_enrollCtx.scanSlotCursor = 1;
  {
    ScopedLock lock(g_r503Mutex, 50);
    if (lock) finger.LEDcontrol(FINGERPRINT_LED_BREATHING, 100, FINGERPRINT_LED_BLUE, 0);
  }
  triggerLedAndBuzzer(2, 1200, BUZZ_ALERT, nowMs);
  triggerPopup("ENROLL GAGAL", reasonMsg, TFT_RED, 1800);
  setFingerStatus(reasonMsg, true);
}

static bool startEnrollSession(unsigned long nowMs) {
  if (!isSystemUnlockedAtomic()) {
    triggerPopup("TERKUNCI", "BUKA HUD DULU", TFT_RED, 1500);
    return false;
  }

  {
    ScopedLock lock(g_r503Mutex, 100);
    if (!lock) {
      abortEnrollSession("R503 SIBUK", nowMs);
      return false;
    }
    if (!g_r503Ok.load(std::memory_order_acquire)) {
      const bool ok = finger.verifyPassword();
      g_r503Ok.store(ok, std::memory_order_release);
      if (!ok) {
        lock.release();
        abortEnrollSession("R503 OFFLINE", nowMs);
        return false;
      }
      applyR503DefaultConfig();
    }
    finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 25, FINGERPRINT_LED_RED, 0);
  }

  g_enrollCtx.state           = ENROLL_FIND_SLOT;
  g_enrollCtx.scanSlotCursor  = 1;
  g_enrollCtx.targetSlot      = 0;
  g_enrollCtx.stepStartMs     = nowMs;
  g_enrollCtx.lastSlotProbeMs = 0;

  setFingerStatus("CARI SLOT R503...", false);
  triggerPopup("ENROLL BARU", "CARI SLOT", TFT_CYAN, 1500);
  triggerLedAndBuzzer(1, 800, BUZZ_ENROLL_STEP, nowMs);
  return true;
}

static const char *enrollTimeoutReason(EnrollFsmState st) {
  switch (st) {
    case ENROLL_FIND_SLOT:     return "TMO SLOT R503";
    case ENROLL_WAIT_FINGER_1: return "TMO JARI 1";
    case ENROLL_WAIT_LIFT:     return "TMO ANGKAT";
    default:                   return "TMO JARI 2";
  }
}

static void stepEnrollFindSlot(unsigned long nowMs) {
  if ((nowMs - g_enrollCtx.lastSlotProbeMs) < ENROLL_SLOT_PROBE_GAP_MS) return;
  g_enrollCtx.lastSlotProbeMs = nowMs;

  bool foundFree = false;
  bool exhausted = false;
  uint16_t freeSlot = 0;
  {
    ScopedLock lock(g_r503Mutex, 50);
    if (!lock) return;

    for (uint16_t checked = 0; checked < ENROLL_SLOTS_PER_TICK && g_enrollCtx.scanSlotCursor <= FINGER_MAX_SLOTS; checked++) {
      const uint16_t candidateId = g_enrollCtx.scanSlotCursor++;
      if (finger.loadModel(candidateId) != FINGERPRINT_OK) {
        freeSlot  = candidateId;
        foundFree = true;
        break;
      }
    }
    exhausted = (!foundFree && g_enrollCtx.scanSlotCursor > FINGER_MAX_SLOTS);
  }

  if (foundFree) {
    g_enrollCtx.state       = ENROLL_WAIT_FINGER_1;
    g_enrollCtx.targetSlot  = freeSlot;
    g_enrollCtx.stepStartMs = nowMs;

    char popL2[20];
    snprintf(popL2, sizeof(popL2), "SLOT #%u", freeSlot);
    setFingerStatus("TEMPEL JARI 1/2", false);
    triggerPopup("ENROLL 1/2", popL2, TFT_YELLOW, 2200);
    triggerLedAndBuzzer(2, 1800, BUZZ_ENROLL_STEP, nowMs);
  } else if (exhausted) {
    abortEnrollSession("MEMORI PENUH", nowMs);
  }
}

static void stepEnrollCapture(unsigned long nowMs) {
  ScopedLock lock(g_r503Mutex, 50);
  if (!lock) return;

  switch (g_enrollCtx.state) {
    case ENROLL_WAIT_FINGER_1: {
      const uint8_t p = finger.getImage();
      if (p == FINGERPRINT_OK) {
        if (finger.image2Tz(1) != FINGERPRINT_OK) {
          lock.release();
          abortEnrollSession("CONV #1 GAGAL", nowMs);
          return;
        }
        g_enrollCtx.state       = ENROLL_WAIT_LIFT;
        g_enrollCtx.stepStartMs = nowMs;
        finger.LEDcontrol(FINGERPRINT_LED_OFF, 0, 0, 0);
        lock.release();

        setFingerStatus("ANGKAT JARI...", false);
        triggerLedAndBuzzer(1, 500, BUZZ_ENROLL_STEP, nowMs);
        triggerPopup("ANGKAT JARI", "TUNGGU...", TFT_CYAN, 2000);
      } else if (p != FINGERPRINT_NOFINGER) {
        lock.release();
        abortEnrollSession("SCAN #1 GAGAL", nowMs);
      }
      break;
    }

    case ENROLL_WAIT_LIFT: {
      if (finger.getImage() == FINGERPRINT_NOFINGER) {
        g_enrollCtx.state       = ENROLL_WAIT_FINGER_2;
        g_enrollCtx.stepStartMs = nowMs;
        finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 25, FINGERPRINT_LED_RED, 0);
        lock.release();

        setFingerStatus("TEMPEL JARI 2/2", false);
        triggerPopup("ENROLL 2/2", "TEMPEL LAGI", TFT_YELLOW, 2200);
        triggerLedAndBuzzer(2, 1800, BUZZ_ENROLL_STEP, nowMs);
      }
      break;
    }

    case ENROLL_WAIT_FINGER_2: {
      const uint8_t p = finger.getImage();
      if (p == FINGERPRINT_OK) {
        if (finger.image2Tz(2) != FINGERPRINT_OK) {
          lock.release();
          abortEnrollSession("CONV #2 GAGAL", nowMs);
          return;
        }
        if (finger.createModel() != FINGERPRINT_OK) {
          lock.release();
          abortEnrollSession("TDK COCOK", nowMs);
          return;
        }

        const uint16_t savedSlot = g_enrollCtx.targetSlot;
        const uint8_t storeRes   = finger.storeModel(savedSlot);
        g_enrollCtx.state          = ENROLL_IDLE;
        g_enrollCtx.scanSlotCursor = 1;

        if (storeRes == FINGERPRINT_OK) {
          finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 10, FINGERPRINT_LED_BLUE, 3);
          lock.release();

          char okLcd[32];
          snprintf(okLcd, sizeof(okLcd), "TERSIMPAN #%u", savedSlot);
          setFingerStatus(okLcd, true);
          triggerPopup("TERSIMPAN", okLcd, TFT_GREEN, 2000);
          triggerLedAndBuzzer(1, 1500, BUZZ_ENROLL_OK, nowMs);
        } else {
          finger.LEDcontrol(FINGERPRINT_LED_BREATHING, 100, FINGERPRINT_LED_BLUE, 0);
          lock.release();
          abortEnrollSession("GAGAL SIMPAN", nowMs);
        }
      } else if (p != FINGERPRINT_NOFINGER) {
        lock.release();
        abortEnrollSession("SCAN #2 GAGAL", nowMs);
      }
      break;
    }

    default:
      break;
  }
}

static void stepEnrollSession(unsigned long nowMs) {
  if (g_enrollCtx.state == ENROLL_IDLE) return;
  if ((nowMs - g_enrollCtx.stepStartMs) >= ENROLL_STEP_TMO_MS) {
    abortEnrollSession(enrollTimeoutReason(g_enrollCtx.state), nowMs);
    return;
  }
  if (g_enrollCtx.state == ENROLL_FIND_SLOT) stepEnrollFindSlot(nowMs);
  else                                        stepEnrollCapture(nowMs);
}

// ============================================================================
// ESP-NOW TCI
// ============================================================================
static void applyEspNowRadioParams(bool disconnectStaFirst) {
  if (disconnectStaFirst) WiFi.disconnect(false, false);
  esp_wifi_set_mac(WIFI_IF_STA, HUD_S3_CUSTOM_MAC);
  esp_wifi_set_channel(TCI_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

static void recordEspSendResult(bool ok) {
  CritSection cs(g_stateMux);
  g_lastEspSendOk = ok;
  g_espSendCbSeen = true;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onTciEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  recordEspSendResult(status == ESP_NOW_SEND_SUCCESS);
}
#else
static void onTciEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  recordEspSendResult(status == ESP_NOW_SEND_SUCCESS);
}
#endif

static esp_err_t addEspNowPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = TCI_WIFI_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer);
}

static void syncPendingEspNowPeer() {
  if (!g_tciEspNowReady.load(std::memory_order_acquire)) return;

  bool hasPending = false;
  uint8_t macCopy[6] = {0};
  {
    CritSection cs(g_stateMux);
    if (g_newPeerPending) {
      hasPending = true;
      memcpy(macCopy, g_pendingPeerMac, 6);
      g_newPeerPending = false;
    }
  }
  if (!hasPending) return;

  if (!esp_now_is_peer_exist(macCopy)) addEspNowPeer(macCopy);
  saveC3MacToNvs(macCopy);
}

static bool sendModePacket(uint8_t mode) {
  if (!isUnlockedAndOnTciPage() || !g_tciEspNowReady.load(std::memory_order_acquire)) {
    return false;
  }

  CommandData cmd;
  cmd.header        = 0xCC55;
  cmd.requestedMode = mode;
  cmd.crc16         = calculateCRC16(reinterpret_cast<const uint8_t*>(&cmd), sizeof(CommandData) - sizeof(uint16_t));

  uint8_t destMac[6];
  {
    CritSection cs(g_stateMux);
    g_espSendCbSeen = false;
    g_lastEspSendOk = false;
    memcpy(destMac, g_c3MacLocked ? activeC3PeerMac : DEFAULT_C3_MAC, 6);
  }

  const esp_err_t res = esp_now_send(destMac, reinterpret_cast<const uint8_t*>(&cmd), sizeof(CommandData));
  const unsigned long nowMs = millis();
  {
    CritSection cs(g_stateMux);
    g_lastModeSendMs = nowMs;
  }
  return (res == ESP_OK);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onTciEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !info->src_addr || !incomingData) return;
  const uint8_t *mac = info->src_addr;
#else
static void onTciEspNowRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (!mac || !incomingData) return;
#endif
  if (!g_tciRxEnabled.load(std::memory_order_acquire)) return;
  if (len != static_cast<int>(sizeof(TelemetryData))) return;
  if (g_espNowRxQueue == nullptr) return;

  EspNowRawFrame frame;
  memcpy(frame.srcMac, mac, 6);
  memcpy(frame.payload, incomingData, sizeof(TelemetryData));
  frame.len  = len;
  frame.rxMs = millis();
  xQueueSend(g_espNowRxQueue, &frame, 0);
}

static bool acceptTciPeer(const uint8_t srcMac[6]) {
  bool accepted = false;
  CritSection cs(g_stateMux);
  if (!STRICT_C3_PEER_LOCK || !g_c3MacLocked) {
    if (memcmp(activeC3PeerMac, srcMac, 6) != 0) {
      if (ALLOW_INITIAL_C3_LEARN || !g_c3MacLocked) {
        memcpy(activeC3PeerMac, srcMac, 6);
        memcpy(g_pendingPeerMac, srcMac, 6);
        g_newPeerPending = true;
        accepted = true;
      }
    } else {
      accepted = true;
    }
    if (STRICT_C3_PEER_LOCK && accepted) g_c3MacLocked = true;
  } else {
    accepted = (memcmp(activeC3PeerMac, srcMac, 6) == 0);
  }
  return accepted;
}

static void processIncomingEspNowFrames() {
  if (g_espNowRxQueue == nullptr) return;

  EspNowRawFrame frame;
  while (xQueueReceive(g_espNowRxQueue, &frame, 0) == pdTRUE) {
    if (!g_tciRxEnabled.load(std::memory_order_acquire) || !isRelayAuthorizedAtomic()) continue;
    if (frame.len != static_cast<int>(sizeof(TelemetryData))) continue;

    const TelemetryData *pkt = reinterpret_cast<const TelemetryData*>(frame.payload);
    if (pkt->header != 0xAA55) continue;
    if (pkt->crc16 != calculateCRC16(frame.payload, sizeof(TelemetryData) - sizeof(uint16_t))) continue;
    if (!acceptTciPeer(frame.srcMac)) continue;

    const int degClamped = constrain(static_cast<int>(pkt->degree10), 0, 350);
    const uint8_t mapped = static_cast<uint8_t>(map(degClamped, 0, 350, 0, 60));

    CritSection cs(g_tciMux);
    memcpy(&g_tciPkt, pkt, sizeof(TelemetryData));
    g_lastTciRxMs = frame.rxMs;
    g_tciRxSeq++;
    g_advHistory[g_advHistIdx] = mapped;
    g_advHistIdx = (g_advHistIdx + 1) % ADV_HIST_LEN;
  }
}

static void initTciEspNowOnce() {
  if (g_tciEspNowReady.load(std::memory_order_acquire)) return;

  WiFi.mode(WIFI_STA);
  applyEspNowRadioParams(true);

  if (esp_now_init() != ESP_OK) return;

  esp_now_register_recv_cb(onTciEspNowRecv);
  esp_now_register_send_cb(onTciEspNowSent);

  const bool basePeerOk = (addEspNowPeer(DEFAULT_C3_MAC) == ESP_OK);

  uint8_t savedMac[6];
  bool hasLocked;
  {
    CritSection cs(g_stateMux);
    hasLocked = g_c3MacLocked;
    memcpy(savedMac, activeC3PeerMac, 6);
  }
  if (hasLocked && memcmp(savedMac, DEFAULT_C3_MAC, 6) != 0 && !esp_now_is_peer_exist(savedMac)) {
    addEspNowPeer(savedMac);
  }

  if (basePeerOk) g_tciEspNowReady.store(true, std::memory_order_release);
  else            esp_now_deinit();
}

static void syncTelemetryGate(PageId page) {
  const bool onTciPage = (page == PAGE_TCI);
  if (!onTciPage) {
    CritSection cs(g_stateMux);
    g_pendingModeChange = false;
    g_newPeerPending    = false;
  }
  g_tciRxEnabled.store(isRelayAuthorizedAtomic(), std::memory_order_release);
  g_allowChronosNotif.store(!onTciPage, std::memory_order_release);
}

// ============================================================================
// SINKRONISASI RTC, NAVIGASI & NOTIFIKASI CHRONOS
// ============================================================================
static void syncRtcFromChronos() {
  static bool wasChronosConnected = false;

  const bool connected = watch.isConnected();
  if (!connected) {
    wasChronosConnected = false;
    return;
  }

  const bool justConnected = !wasChronosConnected;
  wasChronosConnected = true;

  const int year   = watch.getYear();
  const int month  = watch.getMonth() + 1;
  const int day    = watch.getDay();
  const int hour   = watch.getHour(true);
  const int minute = watch.getMinute();
  const int second = watch.getSecond();

  if (year < 2024 || year > 2099 ||
      month < 1 || month > 12 ||
      day < 1 || day > 31 ||
      hour < 0 || hour > 23 ||
      minute < 0 || minute > 59 ||
      second < 0 || second > 59) {
    return;
  }

  const ::DateTime chronosDt(
    static_cast<uint16_t>(year),
    static_cast<uint8_t>(month),
    static_cast<uint8_t>(day),
    static_cast<uint8_t>(hour),
    static_cast<uint8_t>(minute),
    static_cast<uint8_t>(second)
  );

  if (!g_rtcOk.load(std::memory_order_acquire)) return;

  bool needAdjust = justConnected || g_rtcLostPower.load(std::memory_order_acquire);
  {
    ScopedLock lock(g_i2cMutex, 100);
    if (!lock) return;

    if (!needAdjust) {
      const ::DateTime rtcNow = rtc.now();
      if (rtcNow.year() >= 2020 && rtcNow.year() <= 2099) {
        const int32_t diffSec = static_cast<int32_t>(chronosDt.unixtime()) - static_cast<int32_t>(rtcNow.unixtime());
        if (diffSec > 2 || diffSec < -2) needAdjust = true;
      } else {
        needAdjust = true;
      }
    }

    if (needAdjust) {
      rtc.adjust(chronosDt);
      g_rtcLostPower.store(false, std::memory_order_release);
    }
  }

  if (needAdjust) {
    updateRtcCache();
    updateAdaptiveBacklight();
  }
}

struct NavKeywordRule {
  NavTurnType        turn;
  const char *const *words;
  uint8_t            count;
};

template <size_t N>
static constexpr NavKeywordRule makeRule(NavTurnType t, const char *const (&w)[N]) {
  return {t, w, static_cast<uint8_t>(N)};
}

static constexpr const char *KW_ARRIVE[]       = {"arrive", "destination", "tiba", "tujuan", "sampai"};
static constexpr const char *KW_U_TURN[]       = {"u-turn", "uturn", "u turn", "putar balik", "balik arah"};
static constexpr const char *KW_ROUNDABOUT[]   = {"roundabout", "rotary", "bundaran"};
static constexpr const char *KW_SLIGHT_LEFT[]  = {"slight left", "keep left", "serong kiri", "ambil kiri", "tetap di kiri"};
static constexpr const char *KW_SLIGHT_RIGHT[] = {"slight right", "keep right", "serong kanan", "ambil kanan", "tetap di kanan"};
static constexpr const char *KW_LEFT[]         = {"left", "kiri"};
static constexpr const char *KW_RIGHT[]        = {"right", "kanan"};

static constexpr NavKeywordRule NAV_RULES[] = {
  makeRule(NAV_TURN_ARRIVE,       KW_ARRIVE),
  makeRule(NAV_TURN_U_TURN,       KW_U_TURN),
  makeRule(NAV_TURN_ROUNDABOUT,   KW_ROUNDABOUT),
  makeRule(NAV_TURN_SLIGHT_LEFT,  KW_SLIGHT_LEFT),
  makeRule(NAV_TURN_SLIGHT_RIGHT, KW_SLIGHT_RIGHT),
  makeRule(NAV_TURN_LEFT,         KW_LEFT),
  makeRule(NAV_TURN_RIGHT,        KW_RIGHT),
};

static NavTurnType classifyNavTurnFromText(const char *dirText, const char *titleText) {
  const char *parts[2] = {dirText ? dirText : "", titleText ? titleText : ""};
  for (const char *text : parts) {
    for (const NavKeywordRule &rule : NAV_RULES) {
      for (uint8_t i = 0; i < rule.count; i++) {
        if (containsIgnoreCase(text, rule.words[i])) return rule.turn;
      }
    }
  }
  return NAV_TURN_STRAIGHT;
}

static bool isDistanceToken(const char *s, char *outBuf, size_t outSize) {
  if (!s || s[0] == '\0') return false;
  size_t len = strlen(s);
  for (size_t i = 0; i < len; i++) {
    if (isdigit(static_cast<unsigned char>(s[i]))) {
      if (i > 0 && isalpha(static_cast<unsigned char>(s[i - 1]))) continue;
      const char *p = s + i;
      const char *endNum = p;
      while (*endNum && (isdigit(static_cast<unsigned char>(*endNum)) || *endNum == '.' || *endNum == ',')) {
        endNum++;
      }
      const char *u = endNum;
      while (*u == ' ') u++;

      char unit[12] = "";
      size_t uIdx = 0;
      while (*u && isalpha(static_cast<unsigned char>(*u)) && uIdx < sizeof(unit) - 1) {
        unit[uIdx++] = static_cast<char>(tolower(static_cast<unsigned char>(*u)));
        u++;
      }
      unit[uIdx] = '\0';

      if (strcmp(unit, "m") == 0 || strcmp(unit, "meter") == 0 || strcmp(unit, "meters") == 0 ||
          strcmp(unit, "km") == 0 || strcmp(unit, "kilometer") == 0 || strcmp(unit, "kilometers") == 0) {
        size_t numLen = static_cast<size_t>(endNum - p);
        if (numLen >= 8) numLen = 7;
        char numPart[8];
        memcpy(numPart, p, numLen);
        numPart[numLen] = '\0';
        for (size_t k = 0; k < numLen; k++) {
          if (numPart[k] == ',') numPart[k] = '.';
        }
        const char *shortUnit = (unit[0] == 'k') ? "KM" : "M";
        snprintf(outBuf, outSize, "%s %s", numPart, shortUnit);
        return true;
      }
    }
  }
  return false;
}

static uint32_t parseDistanceMeters(const char *distText) {
  if (!distText || distText[0] == '\0') return 999999999UL;
  bool hasVal = false;
  char numBuf[16];
  size_t nIdx = 0;
  size_t pos = 0;
  for (; distText[pos] != '\0' && nIdx < sizeof(numBuf) - 1; pos++) {
    char c = distText[pos];
    if (isdigit(static_cast<unsigned char>(c)) || c == '.') {
      numBuf[nIdx++] = c;
      hasVal = true;
    } else if (c == ',') {
      numBuf[nIdx++] = '.';
      hasVal = true;
    } else if (hasVal) {
      break;
    }
  }
  numBuf[nIdx] = '\0';
  if (!hasVal) return 999999999UL;
  double val = atof(numBuf);
  if (val < 0.0) return 999999999UL;

  while (distText[pos] == ' ') pos++;
  bool isKm = (tolower(static_cast<unsigned char>(distText[pos])) == 'k' &&
               tolower(static_cast<unsigned char>(distText[pos + 1])) == 'm');
  return isKm ? static_cast<uint32_t>(val * 1000.0) : static_cast<uint32_t>(val);
}

static const char* classifyProximityCue(uint32_t distMeters, NavTurnType turn) {
  if (turn == NAV_TURN_ARRIVE) return (distMeters <= 40) ? "TIBA DI TUJUAN" : "MENUJU TUJUAN";
  if (distMeters <= 35)  return "BELOK SEKARANG!";
  if (distMeters <= 120) return "DEKAT (<120M) - SIAP";
  if (distMeters <= 350) return "BERSIAP BELOK";
  return "IKUTI JALUR";
}

static const char* navTurnDefaultLabel(NavTurnType t) {
  switch (t) {
    case NAV_TURN_STRAIGHT:     return "LURUS";
    case NAV_TURN_LEFT:         return "BELOK KIRI";
    case NAV_TURN_RIGHT:        return "BELOK KANAN";
    case NAV_TURN_SLIGHT_LEFT:  return "SERONG KIRI";
    case NAV_TURN_SLIGHT_RIGHT: return "SERONG KANAN";
    case NAV_TURN_U_TURN:       return "PUTAR BALIK";
    case NAV_TURN_ROUNDABOUT:   return "BUNDARAN";
    case NAV_TURN_ARRIVE:       return "TIBA TUJUAN";
    default:                    return "STANDBY";
  }
}

static inline bool isNumChar(char ch) {
  return isdigit(static_cast<unsigned char>(ch)) || ch == '.' || ch == ',';
}

static bool findSpeedNumber(const char *c, char *outBuf, size_t outSize) {
  static constexpr const char *units[3] = {"km/h", "km/j", "kpj"};
  const int len = static_cast<int>(strlen(c));

  for (const char *unit : units) {
    const size_t uLen = strlen(unit);
    for (int p = 1; p < len; p++) {
      bool match = true;
      for (size_t k = 0; k < uLen; k++) {
        if (c[p + k] == '\0' || tolower(static_cast<unsigned char>(c[p + k])) != unit[k]) {
          match = false;
          break;
        }
      }
      if (!match || isalpha(static_cast<unsigned char>(c[p + uLen]))) continue;

      int endI = p - 1;
      while (endI >= 0 && c[endI] == ' ') endI--;
      int startI = endI;
      bool hasDigit = false;
      while (startI >= 0 && isNumChar(c[startI])) {
        if (isdigit(static_cast<unsigned char>(c[startI]))) hasDigit = true;
        startI--;
      }
      startI++;
      while (startI <= endI && (c[startI] == '.' || c[startI] == ',')) startI++;
      while (endI >= startI && (c[endI] == '.' || c[endI] == ',')) endI--;

      if (hasDigit && endI >= startI) {
        char numBuf[10];
        size_t nLen = static_cast<size_t>(endI - startI + 1);
        if (nLen >= sizeof(numBuf)) nLen = sizeof(numBuf) - 1;
        memcpy(numBuf, c + startI, nLen);
        numBuf[nLen] = '\0';
        for (size_t m = 0; m < nLen; m++) {
          if (numBuf[m] == ',') numBuf[m] = '.';
        }
        snprintf(outBuf, outSize, "%s KM/J", numBuf);
        return true;
      }
    }
  }
  return false;
}

static void extractSpeedInfo(const char *speedSrc, const char *titleSrc, const char *dirSrc, char *outBuf, size_t outSize) {
  const char *cands[3] = {speedSrc, titleSrc, dirSrc};
  for (int i = 0; i < 3; i++) {
    const char *c = cands[i];
    if (!c || c[0] == '\0') continue;
    if (findSpeedNumber(c, outBuf, outSize)) return;
    if (i == 0) {
      bool validNum = true;
      bool hasDigit = false;
      for (size_t k = 0; c[k] != '\0'; k++) {
        const unsigned char ch = static_cast<unsigned char>(c[k]);
        if (isdigit(ch)) hasDigit = true;
        else if (ch != '.' && ch != ',') { validNum = false; break; }
      }
      if (validNum && hasDigit) {
        snprintf(outBuf, outSize, "%s KM/J", c);
        return;
      }
    }
  }
  snprintf(outBuf, outSize, "-- KM/J");
}

static bool navContentEqual(const NavTelemetryData &a, const NavTelemetryData &b) {
  return a.bleConnected == b.bleConnected && a.navActive == b.navActive && a.turn == b.turn &&
         strcmp(a.turnLabel, b.turnLabel) == 0 && strcmp(a.distance, b.distance) == 0 &&
         strcmp(a.speedInfo, b.speedInfo) == 0 && strcmp(a.roadName, b.roadName) == 0 &&
         strcmp(a.etaInfo, b.etaInfo) == 0 && strcmp(a.durationInfo, b.durationInfo) == 0;
}

static void updateNavFromChronos() {
  const bool bleConn = watch.isConnected();
  const Navigation nav = watch.getNavigation();

  if (!(bleConn && nav.active)) {
    CritSection cs(g_navMux);
    const bool changed = (g_navData.bleConnected != bleConn) || g_navData.navActive;
    g_navData.bleConnected = bleConn;
    g_navData.navActive    = false;
    if (changed) g_navData.seq++;
    return;
  }

  char dirBuf[48], titleBuf[48], distBuf[20], etaBuf[28], durBuf[22], spdRaw[20];
  copyCleanAscii(dirBuf,   nav.directions.c_str());
  copyCleanAscii(titleBuf, nav.title.c_str());
  copyCleanAscii(distBuf,  nav.distance.c_str());
  copyCleanAscii(etaBuf,   nav.eta.c_str());
  copyCleanAscii(durBuf,   nav.duration.c_str());
  copyCleanAscii(spdRaw,   nav.speed.c_str());

  NavTelemetryData nextNav;
  memset(&nextNav, 0, sizeof(nextNav));
  nextNav.bleConnected = bleConn;
  nextNav.navActive    = true;
  nextNav.turn         = classifyNavTurnFromText(dirBuf, titleBuf);
  copyStr(nextNav.turnLabel, navTurnDefaultLabel(nextNav.turn));

  char parsedDist[16] = "";
  const char *distSrc = "--";
  if (distBuf[0] != '\0') {
    distSrc = isDistanceToken(distBuf, parsedDist, sizeof(parsedDist)) ? parsedDist : distBuf;
  } else if (isDistanceToken(dirBuf, parsedDist, sizeof(parsedDist)) ||
             isDistanceToken(titleBuf, parsedDist, sizeof(parsedDist))) {
    distSrc = parsedDist;
  }
  copyStr(nextNav.distance, distSrc);
  copyStr(nextNav.roadName, (dirBuf[0] != '\0') ? dirBuf : (titleBuf[0] != '\0') ? titleBuf : "Ikuti Rute GMaps");
  copyStr(nextNav.etaInfo,      (etaBuf[0] != '\0') ? etaBuf : "--");
  copyStr(nextNav.durationInfo, (durBuf[0] != '\0') ? durBuf : "--");
  extractSpeedInfo(spdRaw, titleBuf, dirBuf, nextNav.speedInfo, sizeof(nextNav.speedInfo));
  nextNav.lastRxMs = millis();

  CritSection cs(g_navMux);
  nextNav.seq = g_navData.seq;
  if (!navContentEqual(nextNav, g_navData)) nextNav.seq++;
  g_navData = nextNav;
}

static void pollChronosNotification() {
  if (!g_allowChronosNotif.load(std::memory_order_acquire)) return;

  static char lastSig[164] = "";
  static bool wasBleConn = false;

  if (!watch.isConnected()) {
    if (wasBleConn) { lastSig[0] = '\0'; wasBleConn = false; }
    return;
  }
  wasBleConn = true;

  const Notification n = watch.getNotificationAt(0);
  char appBuf[24], titleBuf[32], msgBuf[104];
  copyCleanAscii(appBuf,   n.app.c_str());
  copyCleanAscii(titleBuf, n.title.c_str());
  copyCleanAscii(msgBuf,   n.message.c_str(), false);

  if (appBuf[0] == '\0' && titleBuf[0] == '\0' && msgBuf[0] == '\0') return;

  char sig[sizeof(lastSig)];
  snprintf(sig, sizeof(sig), "%s|%s|%s", appBuf, titleBuf, msgBuf);
  if (strcmp(sig, lastSig) == 0) return;
  copyStr(lastSig, sig);

  NotifPopupData nextNotif;
  memset(&nextNotif, 0, sizeof(nextNotif));
  copyStr(nextNotif.app, appBuf[0] != '\0' ? appBuf : "NOTIF");
  copyStr(nextNotif.title, titleBuf);
  copyStr(nextNotif.message, msgBuf);
  const unsigned long nowMs = millis();
  nextNotif.startMs    = nowMs;
  nextNotif.durationMs = NOTIF_POPUP_MS;
  nextNotif.active     = true;

  {
    CritSection cs(g_notifMux);
    nextNotif.seq = g_notifPopup.seq + 1;
    g_notifPopup  = nextNotif;
  }

  triggerLedAndBuzzer(2, 1500, BUZZ_NOTIF, nowMs);
}

// ============================================================================
// UI LCD
// ============================================================================
enum UiTransitionMode : uint8_t {
  UI_TRANS_NONE = 0,
  UI_TRANS_EXIT_ENTER
};

struct UiTransitionState {
  bool             active;
  UiTransitionMode mode;
  uint8_t          step;
  unsigned long    lastStepMs;
  uint16_t         stepDelayMs;
  bool             exitDone;
  BootState        targetBoot;
  PageId           targetPage;
  uint8_t          targetSub;
};

static UiTransitionState g_uiTrans = {
  false, UI_TRANS_NONE, 0, 0, 8, false, BOOT_WAIT_FINGERPRINT, PAGE_HOME, 0
};

static void resetUiTransition() {
  g_uiTrans.active   = false;
  g_uiTrans.mode     = UI_TRANS_NONE;
  g_uiTrans.step     = 0;
  g_uiTrans.exitDone = false;
}

static void startComponentTransition(BootState bState, PageId page, uint8_t subPage, uint16_t stepDelayMs = 8) {
  g_uiTrans.active      = true;
  g_uiTrans.mode        = UI_TRANS_EXIT_ENTER;
  g_uiTrans.step        = 0;
  g_uiTrans.lastStepMs  = millis();
  g_uiTrans.stepDelayMs = stepDelayMs;
  g_uiTrans.exitDone    = false;
  g_uiTrans.targetBoot  = bState;
  g_uiTrans.targetPage  = page;
  g_uiTrans.targetSub   = subPage;
}

static void animateExitComponents(uint8_t exitPhase) {
  if (exitPhase == 0) {
    lcd.fillRect(0, 0, 320, 85, TFT_BLACK);
    lcd.fillRect(0, 85, 200, 2, TFT_CYAN);
  } else {
    lcd.fillRect(0, 85, 320, 85, TFT_BLACK);
  }
}

static void animateEnterComponentStage(uint8_t enterStage) {
  if (enterStage == 0) lcd.drawFastHLine(0, 24, 320, TFT_CYAN);
  else if (enterStage == 1) lcd.drawFastHLine(8, 86, 304, COL_FRAME);
  else lcd.drawFastHLine(0, 152, 320, TFT_DARKGREY);
}

static void drawTopBar(const char* title, bool relayOn, bool emgBypass = false) {
  GfxBatch batch;
  lcd.fillRect(0, 0, 320, 23, TFT_NAVY);
  lcd.drawFastHLine(0, 23, 320, TFT_CYAN);
  lcd.setTextColor(TFT_WHITE, TFT_NAVY);
  lcd.setTextSize(2);
  lcd.setCursor(8, 4);
  lcd.print(title);

  lcd.setTextSize(1);
  if (emgBypass) {
    lcd.setTextColor(TFT_ORANGE, TFT_NAVY);
    lcd.setCursor(228, 8);
    lcd.print("EMG: BYPASS");
  } else {
    lcd.setTextColor(relayOn ? TFT_GREEN : TFT_RED, TFT_NAVY);
    lcd.setCursor(236, 8);
    lcd.print(relayOn ? "RELAY: ON " : "RELAY: OFF");
  }
}

static void buildTickerText(char *out, size_t outSize, const char *dBuf, bool rtcOk, bool rtcLostPower, const char *fBuf) {
  const char *rtcInfo = !rtcOk ? "NO RTC" : (rtcLostPower ? "SET JAM" : "RTC OK");
  snprintf(out, outSize, "TGL: %s (%s)  |  JARI: %s   ***   ", dBuf, rtcInfo, fBuf);
}

static void drawHomeTopBarWithTicker(bool baseRedraw, bool relayOn, bool emgBypass, const char* tickerText, int tickerOffset) {
  GfxBatch batch;
  if (baseRedraw) {
    lcd.fillRect(0, 0, 320, 23, TFT_NAVY);
    lcd.drawFastHLine(0, 23, 320, TFT_CYAN);
    lcd.setTextColor(TFT_WHITE, TFT_NAVY);
    lcd.setTextSize(2);
    lcd.setCursor(6, 4);
    lcd.print("HOME");

    lcd.setTextSize(1);
    if (emgBypass) {
      lcd.setTextColor(TFT_ORANGE, TFT_NAVY);
      lcd.setCursor(246, 8);
      lcd.print("EMG:BYPASS");
    } else {
      lcd.setTextColor(relayOn ? TFT_GREEN : TFT_RED, TFT_NAVY);
      lcd.setCursor(252, 8);
      lcd.print(relayOn ? "RELAY:ON " : "RELAY:OFF");
    }
  }

  constexpr int winChars = 29;
  char winBuf[winChars + 1];
  int srcLen = static_cast<int>(strlen(tickerText));
  if (srcLen <= 0) {
    memset(winBuf, ' ', winChars);
  } else {
    for (int i = 0; i < winChars; i++) winBuf[i] = tickerText[(tickerOffset + i) % srcLen];
  }
  winBuf[winChars] = '\0';

  lcd.setTextSize(1);
  lcd.setTextColor(TFT_YELLOW, TFT_NAVY);
  lcd.setCursor(62, 8);
  lcd.print(winBuf);
}

static void drawTciPopup(const char* line1, const char* line2, uint16_t bgColor) {
  GfxBatch batch;
  lcd.fillRect(45, 44, 230, 82, bgColor);
  lcd.drawRect(49, 48, 222, 74, TFT_BLACK);
  lcd.setTextColor(TFT_BLACK, bgColor);
  lcd.setTextSize(2);
  lcd.setCursor(60, 58);
  lcd.print(line1);
  lcd.setCursor(60, 88);
  lcd.print(line2);
}

static void drawNotifPopup(const NotifPopupData &n) {
  GfxBatch batch;
  const int boxX = 12, boxY = 26, boxW = 296, boxH = 138;
  const uint16_t hdrCol = containsIgnoreCase(n.app, "wa") ? TFT_DARKGREEN : TFT_NAVY;

  lcd.fillRoundRect(boxX, boxY, boxW, boxH, 6, TFT_BLACK);
  lcd.drawRoundRect(boxX, boxY, boxW, boxH, 6, TFT_CYAN);
  lcd.fillRect(boxX + 2, boxY + 2, boxW - 4, 20, hdrCol);

  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, hdrCol);
  lcd.setCursor(boxX + 8, boxY + 7);
  lcd.printf("APP: %-24s", n.app);

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.setCursor(boxX + 8, boxY + 27);
  char tClip[23];
  copyStr(tClip, n.title);
  lcd.printf("%-22s", tClip);

  lcd.drawFastHLine(boxX + 8, boxY + 46, boxW - 16, TFT_DARKGREY);

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  const int maxCharsPerLine = 22;
  int msgLen = static_cast<int>(strlen(n.message));
  int pos = 0;

  for (int row = 0; row < 3; row++) {
    while (pos < msgLen && (n.message[pos] == ' ' || n.message[pos] == '\r' || n.message[pos] == '\n')) pos++;

    char line[23];
    memset(line, ' ', 22);
    line[22] = '\0';

    int len = 0;
    int lastSpace = -1;
    int scanPos = pos;

    while (scanPos < msgLen && len < maxCharsPerLine) {
      char ch = n.message[scanPos];
      if (ch == '\r' || ch == '\n') break;
      if (ch == ' ') lastSpace = len;
      line[len++] = ch;
      scanPos++;
    }

    int lineChars = len;
    if (scanPos < msgLen && n.message[scanPos] != '\r' && n.message[scanPos] != '\n' && lastSpace > 0) {
      lineChars = lastSpace;
      pos += lastSpace + 1;
    } else {
      pos = scanPos;
      if (pos < msgLen && (n.message[pos] == '\r' || n.message[pos] == '\n')) pos++;
    }

    for (int k = lineChars; k < 22; k++) line[k] = ' ';
    lcd.setCursor(boxX + 8, boxY + 54 + (row * 25));
    lcd.print(line);
  }
}

static void renderBootLockScreen(bool fullClear, bool emgBypass, bool r503Ok, uint8_t lastR503Err, bool rtcOk, bool rtcLostPower, uint8_t enterStage = 255) {
  GfxBatch batch;
  if (fullClear) lcd.fillScreen(TFT_BLACK);

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.drawRoundRect(4, 3, 312, 164, 6, TFT_CYAN);
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(16, 12);
    lcd.print("IMMOBILIZER R503");
    lcd.drawFastHLine(16, 32, 288, TFT_DARKGREY);
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.setCursor(16, 40);
    lcd.print("Jari / GW4 (3M7H) dekat");
    lcd.setCursor(16, 62);
    lcd.print("untuk ON Relay & Sistem ");
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(1);
    lcd.setTextColor(r503Ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    lcd.setCursor(16, 92);
    if (r503Ok) lcd.print("Status R503: TERHUBUNG            ");
    else        lcd.printf("Status R503: ERR (0x%02X) CEK UART  ", lastR503Err);

    lcd.setTextColor(rtcOk ? (rtcLostPower ? TFT_YELLOW : TFT_GREEN) : TFT_RED, TFT_BLACK);
    lcd.setCursor(16, 108);
    if (!rtcOk)            lcd.print("Status RTC : TIDAK TERDETEKSI     ");
    else if (rtcLostPower) lcd.print("Status RTC : OK (BATERAI LEMAH)   ");
    else                   lcd.print("Status RTC : TERHUBUNG            ");

    lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    lcd.setCursor(16, 128);
    lcd.print("Emergency (GPIO2): Tahan 3.5s ON Relay");

    lcd.setTextColor(emgBypass ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(16, 146);
    lcd.print(emgBypass ? "RELAY: ON (EMERGENCY BYPASS AKTIF)  "
                        : "RELAY: OFF (SCAN JARI & GW4 AKTIF)  ");
  }
}

static void renderPageHome(
  bool fullClear,
  bool relayOn,
  bool emgBypass,
  bool rtcOk,
  bool rtcLostPower,
  const char* tBuf,
  const char* dBuf,
  const char* fBuf,
  float chipTempC,
  bool tciOnline,
  uint16_t battVolt10,
  int16_t ntcTemp10,
  int tickerOffset,
  uint8_t enterStage = 255
) {
  GfxBatch batch;
  if (fullClear) lcd.fillScreen(TFT_BLACK);

  char tickerText[96];
  buildTickerText(tickerText, sizeof(tickerText), dBuf, rtcOk, rtcLostPower, fBuf);
  drawHomeTopBarWithTicker(fullClear || enterStage == 0 || enterStage == 255, relayOn, emgBypass, tickerText, tickerOffset);

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.drawRoundRect(6, 28, 168, 96, 6, COL_FRAME);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(14, 34);
    lcd.print("JAM");

    lcd.drawRoundRect(180, 28, 134, 96, 6, COL_FRAME);
    lcd.setCursor(188, 34);
    lcd.print("AKI / VOLT");
  }

  if (fullClear || enterStage == 0 || enterStage == 1 || enterStage == 255) {
    char hhmm[6];
    copyStr(hhmm, tBuf);
    lcd.setTextSize(4);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(14, 56);
    lcd.print(hhmm);

    lcd.setTextSize(2);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(138, 68);
    lcd.print(strlen(tBuf) >= 8 ? (tBuf + 6) : "00");

    if (!tciOnline) {
      lcd.setTextSize(4);
      lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
      lcd.setCursor(186, 56);
      lcd.print("--.-V");
    } else {
      uint16_t voltCol = (battVolt10 < BATT_MIN_WARN_10 || battVolt10 > BATT_MAX_WARN_10) ? TFT_RED : TFT_GREEN;
      lcd.setTextSize(4);
      lcd.setTextColor(voltCol, TFT_BLACK);
      lcd.setCursor(186, 56);
      lcd.printf("%4.1fV", battVolt10 / 10.0f);
    }
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.drawRoundRect(6, 130, 308, 36, 5, COL_FRAME);
    lcd.drawFastVLine(160, 132, 32, TFT_DARKGREY);

    uint16_t chipCol = (chipTempC >= 75.0f) ? TFT_RED : ((chipTempC >= 55.0f) ? TFT_YELLOW : TFT_CYAN);
    lcd.setTextSize(2);
    lcd.setTextColor(chipCol, TFT_BLACK);
    lcd.setCursor(14, 141);
    lcd.printf("MCU:%4.1fC", chipTempC);

    if (!tciOnline) {
      lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
      lcd.setCursor(170, 141);
      lcd.print("MESIN:--C ");
    } else {
      uint16_t ntcCol = (ntcTemp10 >= ENG_TEMP_WARN_10) ? TFT_RED : ((ntcTemp10 > 900) ? TFT_YELLOW : TFT_ORANGE);
      lcd.setTextColor(ntcCol, TFT_BLACK);
      lcd.setCursor(170, 141);
      lcd.printf("MESIN:%3.0fC", ntcTemp10 / 10.0f);
    }
  }
}

static void drawNavTurnIcon(int cx, int cy, NavTurnType turn, uint16_t col) {
  lcd.fillRect(cx - 25, cy - 25, 50, 50, TFT_BLACK);
  lcd.drawRoundRect(cx - 25, cy - 25, 50, 50, 6, TFT_DARKGREY);

  switch (turn) {
    case NAV_TURN_LEFT:
      lcd.fillRect(cx - 4, cy - 4, 14, 16, col);
      lcd.fillRect(cx - 9, cy - 8, 19, 6, col);
      lcd.fillTriangle(cx - 18, cy - 5, cx - 7, cy - 14, cx - 7, cy + 4, col);
      break;
    case NAV_TURN_RIGHT:
      lcd.fillRect(cx - 10, cy - 4, 14, 16, col);
      lcd.fillRect(cx - 10, cy - 8, 19, 6, col);
      lcd.fillTriangle(cx + 18, cy - 5, cx + 7, cy - 14, cx + 7, cy + 4, col);
      break;
    case NAV_TURN_SLIGHT_LEFT:
      lcd.drawLine(cx + 8, cy + 14, cx - 10, cy - 10, col);
      lcd.fillTriangle(cx - 15, cy - 15, cx - 4, cy - 12, cx - 12, cy - 4, col);
      break;
    case NAV_TURN_SLIGHT_RIGHT:
      lcd.drawLine(cx - 8, cy + 14, cx + 10, cy - 10, col);
      lcd.fillTriangle(cx + 15, cy - 15, cx + 4, cy - 12, cx + 12, cy - 4, col);
      break;
    case NAV_TURN_U_TURN:
      lcd.drawArc(cx + 3, cy + 1, 10, 10, 180, 360, col);
      lcd.drawFastVLine(cx + 13, cy + 1, 12, col);
      lcd.drawFastVLine(cx - 7, cy + 1, 8, col);
      lcd.fillTriangle(cx - 7, cy + 16, cx - 13, cy + 8, cx - 1, cy + 8, col);
      break;
    case NAV_TURN_ROUNDABOUT:
      lcd.drawCircle(cx, cy + 1, 10, col);
      lcd.fillTriangle(cx, cy - 17, cx - 6, cy - 8, cx + 6, cy - 8, col);
      break;
    case NAV_TURN_ARRIVE:
      lcd.fillCircle(cx, cy, 11, col);
      lcd.fillCircle(cx, cy, 4, TFT_BLACK);
      break;
    case NAV_TURN_STRAIGHT:
    default:
      lcd.fillRect(cx - 4, cy - 4, 8, 18, col);
      lcd.fillTriangle(cx, cy - 16, cx - 11, cy - 3, cx + 11, cy - 3, col);
      break;
  }
}

static void renderPageNav(bool fullClear, bool relayOn, bool emgBypass, const NavTelemetryData &nav, bool navOnline, uint8_t enterStage = 255) {
  GfxBatch batch;
  if (fullClear) lcd.fillScreen(TFT_BLACK);

  if (fullClear || enterStage == 0 || enterStage == 255) {
    drawTopBar("2. NAVIGASI", relayOn, emgBypass);

    lcd.setTextSize(1);
    lcd.setCursor(70, 27);
    if (navOnline) {
      lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      lcd.print("BLE CHRONOS: GMAPS AKTIF   ");
    } else if (nav.bleConnected) {
      lcd.setTextColor(TFT_CYAN, TFT_BLACK);
      lcd.print("BLE TERHUBUNG (MULAI GMAPS)");
    } else {
      lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
      lcd.print("BLE: MENUNGGU APP CHRONOS  ");
    }

    drawNavTurnIcon(36, 54, navOnline ? nav.turn : NAV_TURN_NONE, navOnline ? TFT_CYAN : TFT_DARKGREY);

    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_WHITE : TFT_SILVER, TFT_BLACK);
    lcd.setCursor(70, 38);
    lcd.printf("%-13s", navOnline ? nav.turnLabel : "NO NAV");

    lcd.setTextColor(navOnline ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(70, 60);
    lcd.printf("%-12s", navOnline ? nav.distance : "--");
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    uint32_t distM = navOnline ? parseDistanceMeters(nav.distance) : 999999999UL;
    const char *cue = navOnline ? classifyProximityCue(distM, nav.turn) : "MENUNGGU RUTE";
    const uint16_t cueCol = !navOnline ? TFT_DARKGREY
                          : (distM <= 35)  ? TFT_RED
                          : (distM <= 120) ? TFT_ORANGE
                          : (distM <= 350) ? TFT_YELLOW : TFT_CYAN;

    lcd.fillRect(8, 82, 304, 28, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setTextColor(cueCol, TFT_BLACK);
    lcd.setCursor(10, 83);
    lcd.print(cue);

    char roadClip[32];
    copyStr(roadClip, navOnline ? nav.roadName
                    : nav.bleConnected ? "Buka rute di Google Maps"
                                       : "Hubungkan HP di Chronos");
    lcd.setTextSize(1);
    lcd.setTextColor(navOnline ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(10, 101);
    lcd.print(roadClip);
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.drawFastHLine(0, 112, 320, TFT_DARKGREY);
    lcd.drawFastVLine(108, 112, 42, TFT_DARKGREY);
    lcd.drawFastVLine(216, 112, 42, TFT_DARKGREY);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(8, 116);
    lcd.print("KECEPATAN");

    char spdClip[9];
    copyStr(spdClip, navOnline ? nav.speedInfo : "-- KM/J");
    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(8, 130);
    lcd.printf("%-8s", spdClip);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(115, 116);
    lcd.print("SISA WAKTU");

    char durClip[9];
    copyStr(durClip, navOnline ? nav.durationInfo : "--");
    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_CYAN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(115, 130);
    lcd.printf("%-8s", durClip);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(223, 116);
    lcd.print("EST. TIBA");

    char etaClip[9];
    copyStr(etaClip, navOnline ? nav.etaInfo : "--");
    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(223, 130);
    lcd.printf("%-8s", etaClip);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(10, 158);
    lcd.print("1x: TCI | 2x: Home | Tahan 3s: Buka Jok");
  }
}

static void renderPageTciDashboard(bool fullClear, bool relayOn, bool emgBypass, const TelemetryData &d, bool online, uint8_t enterStage = 255) {
  GfxBatch batch;
  if (fullClear) lcd.fillScreen(TFT_BLACK);
  if (fullClear || enterStage == 0 || enterStage == 255) drawTopBar("3. TCI DASHBOARD", relayOn, emgBypass);

  if (!online) {
    if (fullClear || enterStage >= 1 || enterStage == 255) {
      lcd.drawRoundRect(10, 28, 300, 134, 6, TFT_RED);
      lcd.fillRect(11, 29, 298, 132, TFT_BLACK);
      lcd.setTextColor(TFT_RED, TFT_BLACK);
      lcd.setTextSize(3);
      lcd.setCursor(65, 58);
      lcd.print("NO SIGNAL");
      lcd.setTextSize(2);
      lcd.setCursor(55, 102);
      lcd.setTextColor(TFT_SILVER, TFT_BLACK);
      lcd.print("Cek C3 / ECU TCI");
    }
    return;
  }

  const char* modeStr = "DAIL";
  uint16_t modeCol = TFT_GREEN;
  if (d.mode == 1)      { modeStr = "RACE"; modeCol = TFT_YELLOW; }
  else if (d.mode == 2) { modeStr = "EXTR"; modeCol = TFT_RED; }
  else if (d.mode == 3) { modeStr = "CUST"; modeCol = TFT_CYAN; }

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(modeCol, TFT_BLACK);
    lcd.setCursor(10, 28);
    lcd.printf("MODE:%-4s", modeStr);

    uint16_t voltCol = (d.battVolt10 < BATT_MIN_WARN_10 || d.battVolt10 > BATT_MAX_WARN_10) ? TFT_RED : TFT_WHITE;
    lcd.setTextColor(voltCol, TFT_BLACK);
    lcd.setCursor(192, 28);
    lcd.printf("%4.1fV", d.battVolt10 / 10.0f);

    lcd.setTextSize(3);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(10, 48);
    lcd.printf("RPM:%5u", d.rpm);

    int barW = map(constrain(static_cast<int>(d.rpm), 0, static_cast<int>(MAX_RPM_SCALE)),
                   0, static_cast<int>(MAX_RPM_SCALE), 0, 298);
    lcd.drawRect(10, 75, 300, 12, TFT_DARKGREY);
    uint16_t barCol = (d.rpm > RPM_WARN_RED) ? TFT_RED : ((d.rpm > RPM_WARN_YELLOW) ? TFT_YELLOW : TFT_GREEN);
    if (barW > 0) lcd.fillRect(11, 76, barW, 10, barCol);
    if (barW < 298) lcd.fillRect(11 + barW, 76, 298 - barW, 10, TFT_BLACK);
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(10, 93);
    lcd.printf("Adv :%5.1f*", d.degree10 / 10.0f);

    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(175, 93);
    lcd.printf("TPS:%3u%%", d.tps);

    uint16_t tempCol = (d.temp10 >= ENG_TEMP_WARN_10) ? TFT_RED : ((d.temp10 > 900) ? TFT_YELLOW : TFT_ORANGE);
    lcd.setTextColor(tempCol, TFT_BLACK);
    lcd.setCursor(10, 114);
    lcd.printf("Temp:%5.0fC", d.temp10 / 10.0f);
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setCursor(10, 135);
    if (d.faultCode != FAULT_NONE || d.state == STATE_LIMP_HOME) {
      lcd.setTextColor(TFT_RED, TFT_BLACK);
      if (d.faultCode & FAULT_TPS_OUT)        lcd.print("LIMP: ERR TPS   ");
      else if (d.faultCode & FAULT_NTC_OUT)   lcd.print("LIMP: ERR SUHU  ");
      else if (d.faultCode & FAULT_BATT_LOW)  lcd.print("LIMP: AKI LOW   ");
      else if (d.faultCode & FAULT_BATT_HIGH) lcd.print("LIMP: OVER VOLT ");
      else if (d.faultCode & FAULT_ADC_ERR)   lcd.print("LIMP: ERR ADC   ");
      else if (d.faultCode & FAULT_NVS_ERR)   lcd.print("WARN: ERR NVS   ");
      else                                    lcd.print("LIMP HOME!      ");
    } else {
      lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      if (d.state == STATE_STOPPED)       lcd.print("ENG: STOP       ");
      else if (d.state == STATE_CRANKING) lcd.print("ENG: CRANK      ");
      else                                lcd.print("ENG: RUN        ");
    }

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(10, 158);
    lcd.print("1x:Kurva | 2x:Home | Tahan 3s:Mode (RPM<=400)");
  }
}

static void renderPageTciCurve(bool fullClear, bool relayOn, bool emgBypass, const TelemetryData &d, const uint8_t *hist, int histIdx, bool online, uint8_t enterStage = 255) {
  GfxBatch batch;
  if (fullClear) lcd.fillScreen(TFT_BLACK);
  if (fullClear || enterStage == 0 || enterStage == 255) drawTopBar("3. ADV CURVE", relayOn, emgBypass);

  constexpr int plotX = 10;
  constexpr int plotY = 28;
  constexpr int plotW = ADV_HIST_LEN;
  constexpr int plotH = 104;
  constexpr int baseY = plotY + plotH - 2;

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.fillRect(plotX + 1, plotY + 1, plotW - 2, plotH - 2, TFT_BLACK);
    lcd.drawRoundRect(plotX, plotY, plotW, plotH, 4, TFT_DARKGREY);

    if (!online) {
      lcd.setTextColor(TFT_RED, TFT_BLACK);
      lcd.setTextSize(2);
      lcd.setCursor(32, 72);
      lcd.print("NO SIGNAL");
      return;
    }

    lcd.drawFastHLine(plotX + 1, plotY + plotH / 2, plotW - 2, COL_FRAME);

    int prevX = plotX + 1;
    int prevY = baseY;
    for (int x = 0; x < ADV_HIST_LEN - 2; x++) {
      int idx = (histIdx + x) % ADV_HIST_LEN;
      int curY = baseY - map(hist[idx], 0, 60, 0, plotH - 6);
      if (x > 0) lcd.drawLine(prevX, prevY, plotX + 1 + x, curY, TFT_GREEN);
      prevX = plotX + 1 + x;
      prevY = curY;
    }
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(180, 34);
    lcd.printf("ADV:%4.1f*", d.degree10 / 10.0f);

    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(180, 62);
    lcd.printf("M:%u", d.mode);

    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.setCursor(180, 90);
    lcd.printf("TPS:%3u%%", d.tps);
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(10, 137);
    lcd.printf("RPM:%5u  TMP:%3.0fC", d.rpm, d.temp10 / 10.0f);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(10, 158);
    lcd.print("1x: Dashboard | 2x: Home | Tahan 3s: Mode");
  }
}

// ============================================================================
// FREERTOS TASKS
// ============================================================================
struct DebouncedButton {
  bool          lastReading  = HIGH;
  bool          stableState  = HIGH;
  unsigned long lastChangeMs = 0;

  bool update(bool reading, unsigned long nowMs) {
    if (reading != lastReading) lastChangeMs = nowMs;
    lastReading = reading;
    if ((nowMs - lastChangeMs) > DEBOUNCE_MS && reading != stableState) {
      stableState = reading;
      return true;
    }
    return false;
  }
  void reset() { lastReading = HIGH; stableState = HIGH; }
};

static void taskImmobilizerCore1(void* pvParameters) {
  (void)pvParameters;
  esp_task_wdt_add(nullptr);

  DebouncedButton mainBtn;
  bool          isPressed      = false;
  bool          longPressSent  = false;
  uint8_t       pendingClicks  = 0;
  unsigned long pressStartMs   = 0;
  unsigned long lastClickRelMs = 0;

  DebouncedButton emBtn;
  bool          emPressed   = false;
  bool          emTriggered = false;
  unsigned long emStart     = 0;

  DebouncedButton enrBtn;
  bool          enrPressed   = false;
  bool          enrTriggered = false;
  unsigned long enrStart     = 0;

  unsigned long lastFingerMs = 0;

  auto resetMainButtonState = [&]() {
    mainBtn.reset();
    isPressed     = false;
    longPressSent = false;
    pendingClicks = 0;
  };

  auto unlockSystem = [&](const char *status, unsigned long nowMs) {
    {
      CritSection cs(g_stateMux);
      copyStr(g_fingerStatus, status);
      g_systemUnlocked = true;
      g_bootState      = BOOT_UNLOCKED_ACTIVE;
      g_currentPage    = PAGE_HOME;
      g_needFullClear  = true;
    }
    requestStopWatchScanOnce();
    applyRelayOutput(nowMs);
    triggerLedAndBuzzer(1, 2000, BUZZ_UNLOCK, nowMs);
    resetMainButtonState();
    postCore0(CMD_PAGE_CHANGED);
  };

  auto goToPage = [&](PageId prev, PageId next, bool unlocked) {
    setSystemStateAtomic(BOOT_UNLOCKED_ACTIVE, next, unlocked, true);
    postCore0(CMD_PAGE_CHANGED, prev, next);
  };

  for (;;) {
    esp_task_wdt_reset();
    const unsigned long nowMs = millis();

    applyRelayOutput(nowMs);

    bool unlockedSnap;
    bool emgSnap;
    {
      CritSection cs(g_stateMux);
      unlockedSnap = g_systemUnlocked;
      emgSnap      = g_emergencyByPass;
    }
    serviceLedAndBuzzer(unlockedSnap || emgSnap, nowMs);

    if (enrBtn.update(digitalRead(PIN_BTN_ENROLL), nowMs)) {
      enrPressed   = (enrBtn.stableState == LOW);
      enrTriggered = false;
      if (enrPressed) enrStart = nowMs;
    }

    if (enrPressed && !enrTriggered && ((nowMs - enrStart) >= ENROLL_BTN_HOLD_MS)) {
      enrTriggered = true;
      if (g_enrollCtx.state != ENROLL_IDLE) abortEnrollSession("BATAL TOMBOL", nowMs);
      else                                  startEnrollSession(nowMs);
    }

    if (g_enrollCtx.state != ENROLL_IDLE) stepEnrollSession(nowMs);

    if (emBtn.update(digitalRead(PIN_BTN_EMRG), nowMs)) {
      emPressed   = (emBtn.stableState == LOW);
      emTriggered = false;
      if (emPressed) emStart = nowMs;
    }

    if (emPressed && !emTriggered && ((nowMs - emStart) >= EMERGENCY_HOLD_MS)) {
      emTriggered = true;
      if (g_enrollCtx.state != ENROLL_IDLE) abortEnrollSession("BATAL (EMG)", nowMs);

      bool newEmg;
      bool stillUnlocked;
      {
        CritSection cs(g_stateMux);
        g_emergencyByPass = !g_emergencyByPass;
        newEmg            = g_emergencyByPass;
        stillUnlocked     = g_systemUnlocked;
        if (newEmg) {
          g_bootState   = BOOT_UNLOCKED_ACTIVE;
          g_currentPage = PAGE_HOME;
          copyStr(g_fingerStatus, "EMERGENCY ON");
        } else if (!stillUnlocked) {
          g_bootState       = BOOT_WAIT_FINGERPRINT;
          g_currentPage     = PAGE_HOME;
          g_seatRelayActive = false;
          copyStr(g_fingerStatus, "LOCKED (R503)");
        } else {
          copyStr(g_fingerStatus, "UNLOCKED (R503)");
        }
        g_needFullClear = true;
      }

      applyRelayOutput(nowMs);
      resetMainButtonState();

      if (newEmg) {
        requestStopWatchScanOnce();
        triggerLedAndBuzzer(2, 2000, BUZZ_ALERT, nowMs);
        triggerPopup("EMERGENCY ON", "RELAY BYPASS", TFT_ORANGE, 1800);
      } else if (!stillUnlocked) {
        triggerLedAndBuzzer(0, 0, BUZZ_ENROLL_STEP, nowMs);
        postCore0(CMD_SYSTEM_LOCKED);
      }
      postCore0(CMD_SAVE_EMG_STATE, PAGE_HOME, PAGE_HOME, newEmg);
    }

    if (!isSystemUnlockedAtomic() && g_watchOneShotMatched.exchange(false, std::memory_order_acq_rel)) {
      char stBuf[32];
      snprintf(stBuf, sizeof(stBuf), "GW4 (%ddBm)", g_watchMatchedRssi.load(std::memory_order_acquire));
      unlockSystem(stBuf, nowMs);
      triggerPopup("WATCH UNLOCK", "RELAY ON", TFT_GREEN, 1800);
    }

    bool inCooldown = g_fingerCooldownActive.load(std::memory_order_acquire);
    if (inCooldown && (nowMs - g_fingerCooldownStartMs.load(std::memory_order_acquire)) >= FINGER_FAIL_COOLDOWN_MS) {
      g_fingerCooldownActive.store(false, std::memory_order_release);
      inCooldown = false;
    }

    if (!isSystemUnlockedAtomic() && (g_enrollCtx.state == ENROLL_IDLE) && !inCooldown &&
        ((nowMs - lastFingerMs) >= FINGER_POLL_MS)) {
      lastFingerMs = nowMs;
      uint16_t fid  = 0;
      int      conf = 0;
      const int scanRes = scanFingerprintOnce(fid, conf);
      if (scanRes == 0) {
        g_fingerFailStreak.store(0, std::memory_order_release);
        char stBuf[32];
        snprintf(stBuf, sizeof(stBuf), "ID#%u (%d)", fid, conf);
        unlockSystem(stBuf, nowMs);
      } else if (scanRes == -2) {
        const uint8_t fails = static_cast<uint8_t>(g_fingerFailStreak.fetch_add(1, std::memory_order_acq_rel) + 1);
        if (fails >= FINGER_MAX_FAIL_STREAK) {
          g_fingerFailStreak.store(0, std::memory_order_release);
          g_fingerCooldownStartMs.store(nowMs, std::memory_order_release);
          g_fingerCooldownActive.store(true, std::memory_order_release);
          triggerLedAndBuzzer(2, 2000, BUZZ_ALERT, nowMs);
          triggerPopup("JARI DITOLAK", "TUNGGU 5 DETIK", TFT_RED, 2200);
        }
      }
    }

    if (isRelayAuthorizedAtomic()) {
      if (mainBtn.update(digitalRead(PIN_BTN_MAIN), nowMs)) {
        if (mainBtn.stableState == LOW) {
          isPressed     = true;
          longPressSent = false;
          pressStartMs  = nowMs;
        } else {
          if (isPressed && !longPressSent) {
            pendingClicks++;
            lastClickRelMs = nowMs;
          }
          if (isSeatRelayActiveAtomic()) setSeatRelayHold(false, nowMs);
          isPressed     = false;
          longPressSent = false;
        }
      }

      if (isPressed && !longPressSent && ((nowMs - pressStartMs) >= LONG_PRESS_MS)) {
        longPressSent = true;
        pendingClicks = 0;
        if (getCurrentPageAtomic() == PAGE_TCI) postCore0(CMD_TCI_NEXT_MODE, PAGE_TCI, PAGE_TCI);
        else                                     setSeatRelayHold(true, nowMs);
      }

      if (pendingClicks > 0 && !isPressed && ((nowMs - lastClickRelMs) >= DOUBLE_TAP_MS)) {
        const uint8_t clicks = pendingClicks;
        pendingClicks = 0;
        const PageId prevPage = getCurrentPageAtomic();
        const bool   curUnl   = isSystemUnlockedAtomic();

        if (clicks >= 2) {
          if (prevPage == PAGE_TCI) {
            goToPage(prevPage, PAGE_HOME, curUnl);
          } else if (prevPage == PAGE_HOME && curUnl) {
            if (g_enrollCtx.state != ENROLL_IDLE) abortEnrollSession("BATAL (LOCK)", nowMs);
            {
              CritSection cs(g_stateMux);
              g_systemUnlocked  = false;
              g_emergencyByPass = false;
              g_bootState       = BOOT_WAIT_FINGERPRINT;
              g_currentPage     = PAGE_HOME;
              g_seatRelayActive = false;
              copyStr(g_fingerStatus, "LOCKED (MANUAL)");
              g_needFullClear   = true;
            }
            applyRelayOutput(nowMs);
            triggerLedAndBuzzer(0, 0, BUZZ_ENROLL_STEP, nowMs);
            postCore0(CMD_SYSTEM_LOCKED, prevPage, PAGE_HOME);
          } else {
            goToPage(prevPage, (prevPage == PAGE_HOME) ? PAGE_NAV : PAGE_HOME, curUnl);
          }
        } else if (prevPage == PAGE_HOME) {
          goToPage(prevPage, PAGE_NAV, curUnl);
        } else if (prevPage == PAGE_NAV) {
          goToPage(prevPage, PAGE_TCI, curUnl);
        } else if (prevPage == PAGE_TCI) {
          postCore0(CMD_SUBPAGE_TOGGLE, PAGE_TCI, PAGE_TCI);
        }
      }
    }

    applyRelayOutput(nowMs);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static const char *const MODE_NAMES[4] = {"DAILY", "RACING", "EXTREME", "CUSTOM"};

static void triggerModePopup(const char *title, uint8_t mode, uint16_t color, uint16_t durationMs) {
  char line2[20];
  snprintf(line2, sizeof(line2), "%s (%u RPM)", MODE_NAMES[mode], static_cast<unsigned int>(getRedlineRPM(mode)));
  triggerPopup(title, line2, color, durationMs);
}

static bool snapshotTciPacket(TelemetryData &snap, unsigned long nowMs, uint32_t *outSeq = nullptr) {
  unsigned long lastRx;
  uint32_t seq;
  {
    CritSection cs(g_tciMux);
    snap   = g_tciPkt;
    lastRx = g_lastTciRxMs;
    seq    = g_tciRxSeq;
  }
  if (outSeq) *outSeq = seq;
  return isTciOnline(lastRx, nowMs);
}

static void handleCore0Command(const Core0Command &cmd, unsigned long nowMs) {
  switch (cmd.type) {
    case CMD_SAVE_EMG_STATE:
      saveEmergencyStateToNvs(cmd.flagVal);
      break;

    case CMD_RESET_C3_MAC:
      clearC3MacFromNvs();
      {
        CritSection cs(g_stateMux);
        memcpy(activeC3PeerMac, DEFAULT_C3_MAC, 6);
        g_c3MacLocked = false;
      }
      triggerPopup("MAC C3 RESET", "PAIRING BARU", TFT_CYAN, 1800);
      break;

    case CMD_PAGE_CHANGED:
      if (cmd.nextPage == PAGE_TCI) {
        initTciEspNowOnce();
        setTciSubPageAtomic(0, false);
      }
      {
        CritSection cs(g_stateMux);
        g_needFullClear = true;
      }
      break;

    case CMD_SUBPAGE_TOGGLE:
      setTciSubPageAtomic(static_cast<uint8_t>((getTciSubPageAtomic() + 1) % 2), true);
      break;

    case CMD_TCI_NEXT_MODE: {
      TelemetryData snap;
      uint32_t rxSeq = 0;
      if (!snapshotTciPacket(snap, nowMs, &rxSeq)) {
        triggerPopup("NO SIGNAL", "C3 / TCI OFFLINE", TFT_RED, 1500);
      } else if (snap.rpm > IDLE_MAX_RPM_MODE) {
        char rpmLine[20];
        snprintf(rpmLine, sizeof(rpmLine), "RPM: %u (>400)", snap.rpm);
        triggerPopup("DITOLAK!", rpmLine, TFT_RED, 1600);
      } else {
        const uint8_t nextM = static_cast<uint8_t>((snap.mode + 1) % 4);
        {
          CritSection cs(g_stateMux);
          g_desiredMode       = nextM;
          g_pendingModeChange = true;
          g_modeRetryCount    = 0;
          g_lastModeSendMs    = 0;
          g_modeReqStartSeq   = rxSeq;
        }
        triggerModePopup("GANTI MODE...", nextM, TFT_YELLOW, 1500);
      }
      break;
    }

    case CMD_SYSTEM_LOCKED:
      setTciSubPageAtomic(0, true);
      break;
  }
}

static void servicePendingModeChange(unsigned long nowMs) {
  bool          pending;
  uint8_t       desiredMode;
  unsigned long lastSendMs;
  uint8_t       retry;
  uint32_t      reqStartSeq;
  bool          sendCbSeen;
  bool          lastSendOk;
  {
    CritSection cs(g_stateMux);
    pending     = g_pendingModeChange;
    desiredMode = g_desiredMode;
    lastSendMs  = g_lastModeSendMs;
    retry       = g_modeRetryCount;
    reqStartSeq = g_modeReqStartSeq;
    sendCbSeen  = g_espSendCbSeen;
    lastSendOk  = g_lastEspSendOk;
  }
  if (!(isUnlockedAndOnTciPage() && pending)) return;

  auto clearPending = [&]() {
    CritSection cs(g_stateMux);
    g_pendingModeChange = false;
  };

  auto retryOrGiveUp = [&](bool clearSendCbFlag, const char *failL1, const char *failL2) {
    bool canRetry;
    {
      CritSection cs(g_stateMux);
      if (clearSendCbFlag) g_espSendCbSeen = false;
      canRetry = (retry < MODE_SEND_MAX_RETRY);
      if (canRetry) g_modeRetryCount = static_cast<uint8_t>(retry + 1);
      else          g_pendingModeChange = false;
    }
    if (canRetry) sendModePacket(desiredMode);
    else          triggerPopup(failL1, failL2, TFT_RED, 1600);
  };

  TelemetryData snap;
  uint32_t curSeq = 0;
  const bool online = snapshotTciPacket(snap, nowMs, &curSeq);
  if (!online || snap.rpm > IDLE_MAX_RPM_MODE) {
    clearPending();
  } else if (curSeq != reqStartSeq && snap.mode == desiredMode && (snap.ackStatus == 1 || snap.ackStatus == 3)) {
    clearPending();
    triggerModePopup("MODE AKTIF:", desiredMode, TFT_GREEN, 1600);
  } else if (sendCbSeen && !lastSendOk) {
    retryOrGiveUp(true, "GAGAL KIRIM", "C3 OFFLINE");
  } else if ((nowMs - lastSendMs) >= MODE_RETRY_INT_MS) {
    retryOrGiveUp(false, "GAGAL MODE", "TMO ACK ECU");
  }
}

static void taskCommCore0(void* pvParameters) {
  (void)pvParameters;
  esp_task_wdt_add(nullptr);

  unsigned long lastNavPollMs   = 0;
  unsigned long lastNotifPollMs = 0;

  for (;;) {
    esp_task_wdt_reset();
    const unsigned long nowMs = millis();

    pollWatchOneShotScan(nowMs);

    const PageId activePage = getCurrentPageAtomic();
    syncTelemetryGate(activePage);

    if (isRelayAuthorizedAtomic() && !g_tciEspNowReady.load(std::memory_order_acquire)) {
      initTciEspNowOnce();
    }

    processIncomingEspNowFrames();
    syncPendingEspNowPeer();

    watch.loop();
    syncRtcFromChronos();

    if (activePage != PAGE_TCI) {
      if ((nowMs - lastNavPollMs) >= NAV_POLL_MS) {
        lastNavPollMs = nowMs;
        updateNavFromChronos();
      }
      if ((nowMs - lastNotifPollMs) >= NOTIF_POLL_MS) {
        lastNotifPollMs = nowMs;
        pollChronosNotification();
      }
    }

    Core0Command cmd;
    while (g_core0CmdQueue && xQueueReceive(g_core0CmdQueue, &cmd, 0) == pdTRUE) {
      handleCore0Command(cmd, nowMs);
    }

    servicePendingModeChange(nowMs);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static constexpr unsigned long UI_FRAME_MS      = 16;
static constexpr unsigned long UI_TRANS_TICK_MS = 8;

static NavTelemetryData snapshotNav(unsigned long nowMs, bool &navOnline) {
  NavTelemetryData snap;
  {
    CritSection cs(g_navMux);
    snap = g_navData;
  }
  navOnline = snap.bleConnected && snap.navActive && (snap.lastRxMs > 0) &&
              ((nowMs - snap.lastRxMs) < NAV_TIMEOUT_MS);
  return snap;
}

static void taskDisplayLcdCore0(void* pvParameters) {
  (void)pvParameters;
  esp_task_wdt_add(nullptr);

  unsigned long lastRtcReadMs    = 0;
  unsigned long lastCurveDrawMs  = 0;
  unsigned long lastTickerStepMs = 0;
  int           homeTickerOffset = 0;

  BootState lastRenderedBoot    = static_cast<BootState>(255);
  PageId    lastRenderedPage    = static_cast<PageId>(255);
  uint8_t   lastRenderedSub     = 255;
  bool      lastRenderedRelay   = false;
  bool      lastRenderedEmg     = false;
  bool      lastRenderedOnline  = false;
  bool      lastRenderedR503    = false;
  uint8_t   lastRenderedR503Err = 255;
  bool      lastRenderedRtc     = false;
  bool      lastRenderedRtcLp   = false;
  bool      lastPopupActive     = false;
  bool      lastNotifActive     = false;
  bool      forceFullRedrawNext = false;
  uint32_t  lastPopupSeq        = 0;
  uint32_t  lastNotifSeq        = 0;
  uint32_t  lastRtcSeq          = 0;
  uint32_t  lastTciSeq          = 0;
  uint32_t  lastNavSeq          = 0;

  for (;;) {
    esp_task_wdt_reset();
    const unsigned long nowMs = millis();

    if ((nowMs - lastRtcReadMs) >= RTC_READ_MS) {
      lastRtcReadMs = nowMs;
      updateRtcCache();
      updateAdaptiveBacklight();
    }

    BootState curBoot;
    PageId    curPage;
    uint8_t   curSub;
    bool      curUnlocked;
    bool      curEmg;
    bool      fullClear;
    char      tBuf[12], dBuf[16], fBuf[32];
    float     chipTempC;
    uint32_t  rtcSeq;
    unsigned long popStart, popDur;
    uint32_t      popSeq;
    char          popL1[20], popL2[20];
    uint16_t      popCol;
    {
      CritSection cs(g_stateMux);
      curBoot     = g_bootState;
      curPage     = g_currentPage;
      curSub      = g_tciSubPage;
      curUnlocked = g_systemUnlocked;
      curEmg      = g_emergencyByPass;
      fullClear   = g_needFullClear || forceFullRedrawNext;
      g_needFullClear     = false;
      forceFullRedrawNext = false;

      memcpy(tBuf, g_timeStr, sizeof(tBuf));
      memcpy(dBuf, g_dateStr, sizeof(dBuf));
      memcpy(fBuf, g_fingerStatus, sizeof(fBuf));
      chipTempC = g_chipTempC;
      rtcSeq    = g_rtcSeq;

      popStart = g_popupStartMs;
      popDur   = g_popupDurationMs;
      popSeq   = g_popupSeq;
      memcpy(popL1, g_popupLine1, sizeof(popL1));
      memcpy(popL2, g_popupLine2, sizeof(popL2));
      popCol   = g_popupBgColor;
    }

    TelemetryData tciSnap;
    unsigned long tciLastRx;
    uint32_t      tciSeq;
    uint8_t       tciHistCopy[ADV_HIST_LEN];
    int           tciHistIdx;
    {
      CritSection cs(g_tciMux);
      tciSnap    = g_tciPkt;
      tciLastRx  = g_lastTciRxMs;
      tciSeq     = g_tciRxSeq;
      memcpy(tciHistCopy, g_advHistory, ADV_HIST_LEN);
      tciHistIdx = g_advHistIdx;
    }
    const bool tciOnline = isTciOnline(tciLastRx, nowMs);

    if (lastRenderedBoot != static_cast<BootState>(255)) {
      if (g_uiTrans.active) {
        if (curBoot != g_uiTrans.targetBoot || curPage != g_uiTrans.targetPage || curSub != g_uiTrans.targetSub) {
          startComponentTransition(curBoot, curPage, curSub, 8);
        }
      } else if (curBoot != lastRenderedBoot || curPage != lastRenderedPage || curSub != lastRenderedSub) {
        startComponentTransition(curBoot, curPage, curSub, 8);
      }
    }

    const bool relayOn     = (curUnlocked || curEmg);
    const bool popupActive = isTimerActive(popStart, popDur, nowMs);

    NotifPopupData notifSnap;
    {
      CritSection cs(g_notifMux);
      if (g_notifPopup.active && !isTimerActive(g_notifPopup.startMs, g_notifPopup.durationMs, nowMs)) {
        g_notifPopup.active = false;
      }
      notifSnap = g_notifPopup;
    }
    const bool notifActive = notifSnap.active && (curBoot == BOOT_UNLOCKED_ACTIVE) && (curPage != PAGE_TCI);

    if (popupActive) {
      resetUiTransition();
      if (!lastPopupActive || popSeq != lastPopupSeq) {
        drawTciPopup(popL1, popL2, popCol);
        lastPopupSeq = popSeq;
      }
      lastPopupActive = true;
      vTaskDelay(pdMS_TO_TICKS(UI_FRAME_MS));
      continue;
    } else if (lastPopupActive) {
      lastPopupActive     = false;
      fullClear           = true;
      forceFullRedrawNext = true;
    }

    if (notifActive) {
      resetUiTransition();
      if (!lastNotifActive || notifSnap.seq != lastNotifSeq) {
        drawNotifPopup(notifSnap);
        lastNotifSeq = notifSnap.seq;
      }
      lastNotifActive = true;
      vTaskDelay(pdMS_TO_TICKS(UI_FRAME_MS));
      continue;
    } else if (lastNotifActive) {
      lastNotifActive     = false;
      fullClear           = true;
      forceFullRedrawNext = true;
    }

    const bool    rtcOk   = g_rtcOk.load(std::memory_order_acquire);
    const bool    rtcLp   = g_rtcLostPower.load(std::memory_order_acquire);
    const bool    r503Ok  = g_r503Ok.load(std::memory_order_acquire);
    const uint8_t r503Err = g_lastR503Err.load(std::memory_order_acquire);

    auto commitCommon = [&](bool online) {
      lastRenderedBoot   = curBoot;
      lastRenderedPage   = curPage;
      lastRenderedRelay  = relayOn;
      lastRenderedEmg    = curEmg;
      lastRenderedOnline = online;
    };

    if (g_uiTrans.active) {
      if ((nowMs - g_uiTrans.lastStepMs) < g_uiTrans.stepDelayMs) {
        vTaskDelay(pdMS_TO_TICKS(UI_TRANS_TICK_MS));
        continue;
      }
      g_uiTrans.lastStepMs = nowMs;

      if (!g_uiTrans.exitDone) {
        animateExitComponents(g_uiTrans.step);
        g_uiTrans.step++;
        if (g_uiTrans.step >= 2) {
          g_uiTrans.exitDone = true;
          g_uiTrans.step     = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(UI_TRANS_TICK_MS));
        continue;
      }

      const uint8_t stage = g_uiTrans.step;
      animateEnterComponentStage(stage);

      if (curBoot == BOOT_WAIT_FINGERPRINT) {
        renderBootLockScreen(false, curEmg, r503Ok, r503Err, rtcOk, rtcLp, stage);
      } else if (curPage == PAGE_HOME) {
        renderPageHome(false, relayOn, curEmg, rtcOk, rtcLp, tBuf, dBuf, fBuf, chipTempC,
                       tciOnline, tciSnap.battVolt10, tciSnap.temp10, homeTickerOffset, stage);
      } else if (curPage == PAGE_NAV) {
        bool navOnline;
        const NavTelemetryData navSnap = snapshotNav(nowMs, navOnline);
        renderPageNav(false, relayOn, curEmg, navSnap, navOnline, stage);
      } else if (curPage == PAGE_TCI) {
        if (curSub == 0) renderPageTciDashboard(false, relayOn, curEmg, tciSnap, tciOnline, stage);
        else             renderPageTciCurve(false, relayOn, curEmg, tciSnap, tciHistCopy, tciHistIdx, tciOnline, stage);
      }

      g_uiTrans.step++;
      if (g_uiTrans.step >= 3) {
        resetUiTransition();
        commitCommon(tciOnline);
        lastRenderedSub = curSub;
        lastTciSeq      = tciSeq;
      }
      vTaskDelay(pdMS_TO_TICKS(UI_TRANS_TICK_MS));
      continue;
    }

    if (curBoot == BOOT_WAIT_FINGERPRINT) {
      const bool changed = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedEmg != curEmg) ||
                           (lastRenderedR503 != r503Ok) || (lastRenderedR503Err != r503Err) ||
                           (lastRenderedRtc != rtcOk) || (lastRenderedRtcLp != rtcLp);
      if (changed) {
        renderBootLockScreen(fullClear, curEmg, r503Ok, r503Err, rtcOk, rtcLp, 255);
        lastRenderedBoot    = curBoot;
        lastRenderedEmg     = curEmg;
        lastRenderedR503    = r503Ok;
        lastRenderedR503Err = r503Err;
        lastRenderedRtc     = rtcOk;
        lastRenderedRtcLp   = rtcLp;
      }
    } else if (curPage == PAGE_HOME) {
      const bool pageChanged = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedPage != curPage) ||
                               (lastRenderedRelay != relayOn) || (lastRenderedEmg != curEmg) ||
                               (lastRenderedRtc != rtcOk) || (lastRenderedRtcLp != rtcLp);
      const bool tickerTick = ((nowMs - lastTickerStepMs) >= TICKER_STEP_MS);
      if (tickerTick) {
        lastTickerStepMs = nowMs;
        homeTickerOffset = (homeTickerOffset + 1) % 256;
      }

      if (pageChanged || rtcSeq != lastRtcSeq || tciSeq != lastTciSeq || tciOnline != lastRenderedOnline) {
        renderPageHome(pageChanged, relayOn, curEmg, rtcOk, rtcLp, tBuf, dBuf, fBuf, chipTempC,
                       tciOnline, tciSnap.battVolt10, tciSnap.temp10, homeTickerOffset, 255);
        commitCommon(tciOnline);
        lastRenderedRtc   = rtcOk;
        lastRenderedRtcLp = rtcLp;
        lastRtcSeq        = rtcSeq;
        lastTciSeq        = tciSeq;
      } else if (tickerTick) {
        char tickerText[96];
        buildTickerText(tickerText, sizeof(tickerText), dBuf, rtcOk, rtcLp, fBuf);
        drawHomeTopBarWithTicker(false, relayOn, curEmg, tickerText, homeTickerOffset);
      }
    } else if (curPage == PAGE_NAV) {
      bool navOnline;
      const NavTelemetryData navSnap = snapshotNav(nowMs, navOnline);
      const bool pageChanged = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedPage != curPage) ||
                               (lastRenderedRelay != relayOn) || (lastRenderedEmg != curEmg);
      if (pageChanged || navSnap.seq != lastNavSeq || navOnline != lastRenderedOnline) {
        renderPageNav(pageChanged, relayOn, curEmg, navSnap, navOnline, 255);
        commitCommon(navOnline);
        lastNavSeq = navSnap.seq;
      }
    } else if (curPage == PAGE_TCI) {
      const bool pageChanged = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedPage != curPage) ||
                               (lastRenderedSub != curSub) || (lastRenderedRelay != relayOn) ||
                               (lastRenderedEmg != curEmg);
      if (pageChanged || tciOnline != lastRenderedOnline || tciSeq != lastTciSeq) {
        bool drawn = false;
        if (curSub == 0) {
          renderPageTciDashboard(pageChanged, relayOn, curEmg, tciSnap, tciOnline, 255);
          drawn = true;
        } else if (pageChanged || tciOnline != lastRenderedOnline || ((nowMs - lastCurveDrawMs) >= CURVE_REDRAW_MS)) {
          lastCurveDrawMs = nowMs;
          renderPageTciCurve(pageChanged, relayOn, curEmg, tciSnap, tciHistCopy, tciHistIdx, tciOnline, 255);
          drawn = true;
        }
        if (drawn) {
          commitCommon(tciOnline);
          lastRenderedSub = curSub;
          lastTciSeq      = tciSeq;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(UI_FRAME_MS));
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  for (const int pin : {PIN_RELAY, PIN_RELAY_SEAT, PIN_LED_NOTIF, PIN_BUZZER}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (const int pin : {PIN_BTN_MAIN, PIN_BTN_EMRG, PIN_BTN_ENROLL}) {
    pinMode(pin, INPUT_PULLUP);
  }

  g_i2cMutex  = xSemaphoreCreateMutex();
  g_nvsMutex  = xSemaphoreCreateMutex();
  g_r503Mutex = xSemaphoreCreateMutex();
  g_core0CmdQueue = xQueueCreate(16, sizeof(Core0Command));
  g_espNowRxQueue = xQueueCreate(16, sizeof(EspNowRawFrame));

  if (!g_i2cMutex || !g_nvsMutex || !g_r503Mutex || !g_core0CmdQueue || !g_espNowRxQueue) {
    delay(1000);
    ESP.restart();
  }

  loadPersistentSettings();

  setSystemStateAtomic(g_emergencyByPass ? BOOT_UNLOCKED_ACTIVE : BOOT_WAIT_FINGERPRINT, PAGE_HOME, false, true);
  applyRelayOutput();
  syncTelemetryGate(PAGE_HOME);

  initLcdBacklightPwm();
  lcd.init();
  lcd.setRotation(1);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(45, 78);
  lcd.print("BOOTING SYSTEM...");

  beginI2c();
  {
    ScopedLock lock(g_i2cMutex, 200);
    if (lock && rtc.begin(&Wire)) {
      g_rtcOk.store(true, std::memory_order_release);
      if (rtc.lostPower()) g_rtcLostPower.store(true, std::memory_order_release);
    }
  }
  updateRtcCache();
  updateAdaptiveBacklight();

  r503Serial.begin(57600, SERIAL_8N1, PIN_R503_RX, PIN_R503_TX);
  {
    ScopedLock lock(g_r503Mutex, 200);
    if (lock) {
      bool r503Ready = false;
      for (int attempt = 0; attempt < 3 && !r503Ready; attempt++) {
        if (finger.verifyPassword()) {
          r503Ready = true;
          applyR503DefaultConfig();
        } else {
          delay(40);
        }
      }
      g_r503Ok.store(r503Ready, std::memory_order_release);
      if (!r503Ready) g_lastR503Err.store(FINGERPRINT_PACKETRECIEVEERR, std::memory_order_release);
    }
  }

  NimBLEDevice::init(BLE_DEVICE_NAME);
  watch.begin();
  watch.set24Hour(true);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdtCfg);
  }
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif

  xTaskCreatePinnedToCore(taskImmobilizerCore1, "ImmobCore1", 5120, nullptr, 4, nullptr, CORE_SECURITY);
  xTaskCreatePinnedToCore(taskCommCore0,        "CommCore0",  6144, nullptr, 3, nullptr, CORE_COMM_UI);
  xTaskCreatePinnedToCore(taskDisplayLcdCore0,  "LcdUiCore0", 8192, nullptr, 2, nullptr, CORE_COMM_UI);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
