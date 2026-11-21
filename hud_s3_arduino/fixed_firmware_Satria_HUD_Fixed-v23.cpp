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
#include <atomic>
#include <cstring>
#include <cctype>
#include "common_proto.h"

// ============================================================================
// KONFIGURASI PIN & HARDWARE
// ============================================================================
constexpr int PIN_BTN_MAIN   = 1;
constexpr int PIN_BTN_EMRG   = 2;
constexpr int PIN_BTN_ENROLL = 38; // GPIO 38 aman (INPUT_PULLUP) untuk pendaftaran sidik jari R503
constexpr int PIN_LED_NOTIF  = 39; // GPIO 39 aman (OUTPUT) untuk LED notifikasi/status
constexpr int PIN_BUZZER     = 40; // GPIO 40 aman (OUTPUT/PWM) untuk buzzer notifikasi
constexpr int PIN_RELAY      = 4;
constexpr int PIN_RELAY_SEAT = 21; // GPIO 21 aman untuk relay pembuka jok elektrik
constexpr int PIN_LCD_BL     = 42; // GPIO 42 aman untuk PWM backlight LCD

constexpr int PIN_I2C_SDA  = 8;
constexpr int PIN_I2C_SCL  = 9;

constexpr int PIN_R503_RX  = 17;
constexpr int PIN_R503_TX  = 18;

// PWM Backlight LCD (Siang & Malam otomatis via RTC)
constexpr uint8_t  LCD_BL_PWM_CH      = 0;
constexpr uint32_t LCD_BL_PWM_FREQ    = 5000;
constexpr uint8_t  LCD_BL_PWM_RES     = 8;
constexpr uint8_t  LCD_BL_DUTY_DAY    = 240; // ~94% kecerahan siang (06:00 - 17:59)
constexpr uint8_t  LCD_BL_DUTY_NIGHT  = 115; // ~45% kecerahan malam (18:00 - 05:59)

// ============================================================================
// KONFIGURASI PROTOKOL, TIMING & AMBANG BATAS (DAILY-USE HARDENED v23)
// ============================================================================
constexpr uint8_t TCI_WIFI_CHANNEL = 1;
static const uint8_t HUD_S3_CUSTOM_MAC[6] = {0x02, 0x53, 0x46, 0x55, 0x48, 0x55};
static const uint8_t DEFAULT_C3_MAC[6]     = {0x02, 0x53, 0x46, 0x55, 0x43, 0x33};

// Kunci MAC peer C3 setelah pairing pertama kali agar aman dari paket asing di parkiran
constexpr bool STRICT_C3_PEER_LOCK = true;

// Izinkan pembelajaran MAC C3 baru hanya jika belum terkunci
constexpr bool ALLOW_INITIAL_C3_LEARN = false; // Nonaktif otomatis setelah lock agar aman dari spoof; aktifkan hanya saat pairing

// Reset emergency bypass saat boot agar relay tidak menyala otomatis tanpa autentikasi ulang
constexpr bool RESET_EMG_ON_BOOT = true;

// Batas percobaan gagal sidik jari sebelum jeda cooldown
constexpr uint8_t       FINGER_MAX_FAIL_STREAK = 5;
constexpr unsigned long FINGER_FAIL_COOLDOWN_MS = 5000;

// Interval polling navigasi & notifikasi Chronos (ms)
constexpr unsigned long NAV_POLL_MS   = 150;
constexpr unsigned long NOTIF_POLL_MS = 150;

constexpr char BLE_DEVICE_NAME[] = "Satria-HUD";
constexpr char NVS_NAMESPACE[]   = "satria_hud";

constexpr BaseType_t CORE_COMM_UI  = 0;
constexpr BaseType_t CORE_SECURITY = 1;

constexpr uint16_t MAX_RPM_SCALE        = 14000;
constexpr uint16_t IDLE_MAX_RPM_MODE    = 400;
constexpr uint16_t RPM_WARN_YELLOW      = 7500;
constexpr uint16_t RPM_WARN_RED         = 10500;

constexpr uint16_t BATT_MIN_WARN_10     = 115; // 11.5V
constexpr uint16_t BATT_MAX_WARN_10     = 155; // 15.5V
constexpr int16_t  ENG_TEMP_WARN_10     = 950; // 95.0C

constexpr unsigned long DEBOUNCE_MS         = 50;
constexpr unsigned long LONG_PRESS_MS       = 3000;
constexpr unsigned long SEAT_MAX_HOLD_MS    = 8000; // Fail-safe maksimal relay jok menyala jika tombol stuck
constexpr unsigned long EMERGENCY_HOLD_MS   = 3500; // Diperpanjang menjadi 3.5s agar tidak tersentuh tak sengaja saat jalan
constexpr unsigned long DOUBLE_TAP_MS       = 300;
constexpr unsigned long TCI_TIMEOUT_MS      = 1500;
constexpr unsigned long NAV_TIMEOUT_MS      = 10000;
constexpr unsigned long NOTIF_POPUP_MS      = 5000;
constexpr unsigned long ESPNOW_RETRY_MS     = 5000;
constexpr unsigned long ENROLL_HOLD_MS      = 2000;
constexpr uint16_t      FINGER_MAX_SLOTS    = 127;

constexpr int      ADV_HIST_LEN         = 160;
constexpr uint32_t WDT_TIMEOUT_SEC      = 5;
constexpr uint32_t I2C_CLOCK_HZ         = 100000;
constexpr uint8_t  MODE_SEND_MAX_RETRY  = 5;

#define FAULT_NONE      0
#define FAULT_TPS_OUT   (1 << 0)
#define FAULT_NTC_OUT   (1 << 1)
#define FAULT_BATT_LOW  (1 << 2)
#define FAULT_BATT_HIGH (1 << 3)
#define FAULT_NVS_ERR   (1 << 4)
#define FAULT_ADC_ERR   (1 << 5)

#define STATE_STOPPED   0
#define STATE_CRANKING  1
#define STATE_RUNNING   2
#define STATE_LIMP_HOME 3

// ============================================================================
// DRIVER LAYAR LOVYANGFX (ILI9341 240x320 -> Rotasi 1 = 320x240)
// ============================================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI       _bus_instance;

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
      cfg.pin_sclk    = 12;
      cfg.pin_mosi    = 11;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = 14;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 10;
      cfg.pin_rst          = 13;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = false;
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
static std::atomic<unsigned long> g_fingerCooldownUntilMs{0};

static ChronosESP32 watch(BLE_DEVICE_NAME);

// ============================================================================
// STRUKTUR PAKET TELEMETRI & PERINTAH
// ============================================================================
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
      if (crc & 0x8000) crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      else crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static uint16_t getRedlineRPM(uint8_t mode) {
  switch (mode) {
    case 0:  return 9000;
    case 1:  return 11000;
    case 2:  return 12000;
    case 3:  return 11500;
    default: return 9000;
  }
}

// ============================================================================
// STRUKTUR NAVIGASI & NOTIFIKASI CHRONOS
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
  false,
  false,
  NAV_TURN_NONE,
  "STANDBY",
  "--",
  "-- KM/J",
  "Buka GMaps & Chronos",
  "--",
  "--",
  0,
  0
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
// STATE SISTEM & SINKRONISASI RTOS
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
  CMD_OPEN_SEAT,
  CMD_SYSTEM_LOCKED,
  CMD_SAVE_EMG_STATE
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

static std::atomic<uint32_t> g_diagCrcErr{0};
static std::atomic<uint32_t> g_diagMacDrop{0};
static std::atomic<uint32_t> g_diagTxFail{0};

static SemaphoreHandle_t g_i2cMutex  = nullptr;
static SemaphoreHandle_t g_nvsMutex  = nullptr;
static SemaphoreHandle_t g_r503Mutex = nullptr;

static portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
static char g_fingerStatus[32] = "R503 Ready";
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
static volatile bool    g_lastEspSendOk     = false;
static volatile bool    g_espSendCbSeen     = false;

static uint8_t          activeC3PeerMac[6]  = {0x02, 0x53, 0x46, 0x55, 0x43, 0x33};
static volatile bool    g_c3MacLocked       = false;
static volatile bool    g_newPeerPending    = false;
static uint8_t          g_pendingPeerMac[6] = {0};

static QueueHandle_t g_core0CmdQueue   = nullptr;
static QueueHandle_t g_espNowRxQueue   = nullptr;

// ============================================================================
// HELPER UTILITAS, PWM BACKLIGHT & NVS PERSISTENCE
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
    if (hr >= 18 || hr < 6) {
      targetDuty = LCD_BL_DUTY_NIGHT;
    }
  }
  if (targetDuty != lastDuty) {
    lastDuty = targetDuty;
    setLcdBacklightDuty(targetDuty);
  }
}

static void saveEmergencyStateToNvs(bool emgState);

static void loadPersistentSettings() {
  if (g_nvsMutex && xSemaphoreTake(g_nvsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool savedEmg = false;
    bool hasMac = false;
    uint8_t savedMac[6] = {0};

    if (prefs.begin(NVS_NAMESPACE, true)) {
      savedEmg = prefs.getBool("emg_bypass", false);
      size_t macLen = prefs.getBytesLength("c3_mac");
      hasMac = (macLen == 6 && prefs.getBytes("c3_mac", savedMac, 6) == 6);
      prefs.end();
    }

    bool activeEmgOnBoot = RESET_EMG_ON_BOOT ? false : savedEmg;

    portENTER_CRITICAL(&g_stateMux);
    g_emergencyByPass = activeEmgOnBoot;
    if (activeEmgOnBoot) {
      memcpy(g_fingerStatus, "EMERGENCY ON", sizeof(g_fingerStatus));
    } else if (savedEmg) {
      memcpy(g_fingerStatus, "LOCKED (EMG RST)", sizeof(g_fingerStatus));
    }
    if (hasMac) {
      memcpy(activeC3PeerMac, savedMac, 6);
      g_c3MacLocked = true;
    }
    portEXIT_CRITICAL(&g_stateMux);

    xSemaphoreGive(g_nvsMutex);

    if (RESET_EMG_ON_BOOT && savedEmg) {
      saveEmergencyStateToNvs(false);
    }
  }
}

static void saveEmergencyStateToNvs(bool emgState) {
  if (g_nvsMutex && xSemaphoreTake(g_nvsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (prefs.begin(NVS_NAMESPACE, false)) {
      prefs.putBool("emg_bypass", emgState);
      prefs.end();
    }
    xSemaphoreGive(g_nvsMutex);
  }
}

static void saveC3MacToNvs(const uint8_t mac[6]) {
  if (g_nvsMutex && xSemaphoreTake(g_nvsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (prefs.begin(NVS_NAMESPACE, false)) {
      prefs.putBytes("c3_mac", mac, 6);
      prefs.end();
    }
    xSemaphoreGive(g_nvsMutex);
  }
}

static void sanitizeAsciiInPlace(char *s, bool replaceNewlineWithSpace = true) {
  if (!s) return;
  size_t w = 0;
  for (size_t r = 0; s[r] != '\0'; r++) {
    unsigned char ch = static_cast<unsigned char>(s[r]);
    if (ch == '\r') continue;
    if (ch == '\n') {
      s[w++] = replaceNewlineWithSpace ? ' ' : '\n';
    } else if (ch >= 0x20 && ch <= 0x7E) {
      s[w++] = static_cast<char>(ch);
    } else {
      s[w++] = '?';
    }
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
  while (s[start] == ' ' || s[start] == '\r' || s[start] == '\n' || s[start] == '\t') {
    start++;
  }
  if (start > 0) {
    memmove(s, s + start, len - start + 1);
  }
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
  portENTER_CRITICAL(&g_stateMux);
  uint8_t sp = g_tciSubPage;
  portEXIT_CRITICAL(&g_stateMux);
  return sp;
}

static void setTciSubPageAtomic(uint8_t sp, bool fullClear) {
  portENTER_CRITICAL(&g_stateMux);
  g_tciSubPage = sp;
  if (fullClear) g_needFullClear = true;
  portEXIT_CRITICAL(&g_stateMux);
}

inline void applyRelayOutput(unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  bool relayOn = false;
  bool seatOn  = false;
  portENTER_CRITICAL(&g_stateMux);
  relayOn = (g_systemUnlocked || g_emergencyByPass);
  if (g_seatRelayActive && ((nowMs - g_seatOpenStartMs) >= SEAT_MAX_HOLD_MS)) {
    g_seatRelayActive = false;
  }
  seatOn = g_seatRelayActive;
  portEXIT_CRITICAL(&g_stateMux);
  digitalWrite(PIN_RELAY, relayOn ? HIGH : LOW);
  digitalWrite(PIN_RELAY_SEAT, seatOn ? HIGH : LOW);
}

void setSeatRelayHold(bool active, unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  bool changed = false;
  portENTER_CRITICAL(&g_stateMux);
  if (g_seatRelayActive != active) {
    g_seatRelayActive = active;
    changed = true;
    if (active) {
      g_seatOpenStartMs = nowMs;
    }
  }
  portEXIT_CRITICAL(&g_stateMux);
  applyRelayOutput(nowMs);
  if (active && changed) {
    triggerPopup("JOK TERBUKA", "LEPAS TOMBOL OFF", TFT_CYAN, 2000);
  }
}

void setSystemStateAtomic(BootState bState, PageId page, bool unlocked, bool fullClear) {
  portENTER_CRITICAL(&g_stateMux);
  g_bootState      = bState;
  g_currentPage    = page;
  g_systemUnlocked = unlocked;
  if (fullClear) {
    g_needFullClear = true;
  }
  portEXIT_CRITICAL(&g_stateMux);
}

void readSystemStateAtomic(BootState &bState, PageId &page, bool &unlocked, bool &emgBypass, bool &fullClear, bool consumeClear = true) {
  portENTER_CRITICAL(&g_stateMux);
  bState    = g_bootState;
  page      = g_currentPage;
  unlocked  = g_systemUnlocked;
  emgBypass = g_emergencyByPass;
  fullClear = g_needFullClear;
  if (consumeClear) {
    g_needFullClear = false;
  }
  portEXIT_CRITICAL(&g_stateMux);
}

static bool isRelayAuthorizedAtomic() {
  portENTER_CRITICAL(&g_stateMux);
  bool ok = (g_systemUnlocked || g_emergencyByPass);
  portEXIT_CRITICAL(&g_stateMux);
  return ok;
}

bool isUnlockedAndOnTciPage() {
  if (!g_tciRxEnabled.load(std::memory_order_acquire)) return false;
  portENTER_CRITICAL(&g_stateMux);
  bool ok = ((g_systemUnlocked || g_emergencyByPass) && g_currentPage == PAGE_TCI);
  portEXIT_CRITICAL(&g_stateMux);
  return ok;
}

void triggerPopup(const char* l1, const char* l2, uint16_t bgColor, uint16_t durationMs = 1500) {
  char buf1[20];
  char buf2[20];
  strncpy(buf1, l1 ? l1 : "", sizeof(buf1) - 1);
  buf1[sizeof(buf1) - 1] = '\0';
  strncpy(buf2, l2 ? l2 : "", sizeof(buf2) - 1);
  buf2[sizeof(buf2) - 1] = '\0';
  unsigned long startMs = millis();

  portENTER_CRITICAL(&g_stateMux);
  memcpy(g_popupLine1, buf1, sizeof(g_popupLine1));
  memcpy(g_popupLine2, buf2, sizeof(g_popupLine2));
  g_popupBgColor    = bgColor;
  g_popupStartMs    = startMs;
  g_popupDurationMs = durationMs;
  g_popupSeq++;
  portEXIT_CRITICAL(&g_stateMux);
}

// ============================================================================
// I2C & RTC DS3231
// ============================================================================
void recoverI2cBus() {
  if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    Serial.println("[I2C] Recovering bus...");
    Wire.end();
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(PIN_I2C_SCL, HIGH);
      delayMicroseconds(5);
      digitalWrite(PIN_I2C_SCL, LOW);
      delayMicroseconds(5);
    }
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    Wire.setTimeOut(50);
    xSemaphoreGive(g_i2cMutex);
  }
}

void updateRtcCache() {
  if (!g_rtcOk.load(std::memory_order_acquire)) return;

  DateTime now;
  bool readOk = false;
  if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    now = rtc.now();
    readOk = true;
    xSemaphoreGive(g_i2cMutex);
  }

  if (!readOk) return;

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
  if (isnan(chipC) || chipC < -20.0f || chipC > 125.0f) {
    chipC = 0.0f;
  }

  portENTER_CRITICAL(&g_stateMux);
  memcpy(g_timeStr, tBuf, sizeof(g_timeStr));
  memcpy(g_dateStr, dBuf, sizeof(g_dateStr));
  g_chipTempC = chipC;
  g_rtcSeq++;
  portEXIT_CRITICAL(&g_stateMux);
}

// ============================================================================
// INDIKATOR LED & BUZZER NOTIFIKASI (NON-BLOCKING)
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
  uint8_t       ledMode; // 0=off, 1=steady, 2=blink fast, 3=pulse slow
  unsigned long ledUntilMs;
  unsigned long ledLastToggleMs;
  bool          ledPinState;

  BuzzerPattern buzzPat;
  uint8_t       buzzStep;
  unsigned long buzzStepStartMs;
  bool          buzzActiveTone;
};

static portMUX_TYPE g_hwNotifMux = portMUX_INITIALIZER_UNLOCKED;
static NotifHardwareState g_hwNotif = {0, 0, 0, false, BUZZ_NONE, 0, 0, false};

static void setBuzzerTonePin(bool on) {
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

static void setLedNotifPin(bool on) {
  digitalWrite(PIN_LED_NOTIF, on ? HIGH : LOW);
}

void triggerLedAndBuzzer(uint8_t ledMode, uint16_t ledDurationMs, BuzzerPattern pat, unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  portENTER_CRITICAL(&g_hwNotifMux);
  if (ledMode > 0) {
    g_hwNotif.ledMode = ledMode;
    g_hwNotif.ledUntilMs = nowMs + ledDurationMs;
    g_hwNotif.ledLastToggleMs = nowMs;
    g_hwNotif.ledPinState = true;
  }
  if (pat != BUZZ_NONE) {
    g_hwNotif.buzzPat = pat;
    g_hwNotif.buzzStep = 0;
    g_hwNotif.buzzStepStartMs = nowMs;
    g_hwNotif.buzzActiveTone = true;
  }
  portEXIT_CRITICAL(&g_hwNotifMux);
}

void serviceLedAndBuzzer(bool systemUnlockedOrEmg, unsigned long nowMs = 0) {
  if (nowMs == 0) nowMs = millis();
  bool outLed = false;
  bool outBuzz = false;

  portENTER_CRITICAL(&g_hwNotifMux);
  if (g_hwNotif.ledMode != 0 && nowMs >= g_hwNotif.ledUntilMs) {
    g_hwNotif.ledMode = 0;
    g_hwNotif.ledPinState = false;
  }

  if (g_hwNotif.ledMode == 1) {
    outLed = true;
  } else if (g_hwNotif.ledMode == 2) {
    if ((nowMs - g_hwNotif.ledLastToggleMs) >= 120) {
      g_hwNotif.ledLastToggleMs = nowMs;
      g_hwNotif.ledPinState = !g_hwNotif.ledPinState;
    }
    outLed = g_hwNotif.ledPinState;
  } else if (g_hwNotif.ledMode == 3) {
    if ((nowMs - g_hwNotif.ledLastToggleMs) >= 450) {
      g_hwNotif.ledLastToggleMs = nowMs;
      g_hwNotif.ledPinState = !g_hwNotif.ledPinState;
    }
    outLed = g_hwNotif.ledPinState;
  } else {
    outLed = systemUnlockedOrEmg;
  }

  BuzzerPattern pat = g_hwNotif.buzzPat;
  uint8_t step = g_hwNotif.buzzStep;
  unsigned long elapsed = nowMs - g_hwNotif.buzzStepStartMs;

  if (pat == BUZZ_UNLOCK) {
    if (step == 0) { outBuzz = true; if (elapsed >= 90)  { g_hwNotif.buzzStep = 1; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 1) { outBuzz = false; if (elapsed >= 60)  { g_hwNotif.buzzStep = 2; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 2) { outBuzz = true; if (elapsed >= 140) { g_hwNotif.buzzPat = BUZZ_NONE; g_hwNotif.buzzStep = 0; outBuzz = false; } }
  } else if (pat == BUZZ_ENROLL_STEP) {
    outBuzz = true;
    if (elapsed >= 80) { g_hwNotif.buzzPat = BUZZ_NONE; g_hwNotif.buzzStep = 0; outBuzz = false; }
  } else if (pat == BUZZ_ENROLL_OK) {
    if (step == 0) { outBuzz = true; if (elapsed >= 80)  { g_hwNotif.buzzStep = 1; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 1) { outBuzz = false; if (elapsed >= 70)  { g_hwNotif.buzzStep = 2; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 2) { outBuzz = true; if (elapsed >= 80)  { g_hwNotif.buzzStep = 3; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 3) { outBuzz = false; if (elapsed >= 70)  { g_hwNotif.buzzStep = 4; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 4) { outBuzz = true; if (elapsed >= 180) { g_hwNotif.buzzPat = BUZZ_NONE; g_hwNotif.buzzStep = 0; outBuzz = false; } }
  } else if (pat == BUZZ_ALERT) {
    if (step == 0) { outBuzz = true; if (elapsed >= 120) { g_hwNotif.buzzStep = 1; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 1) { outBuzz = false; if (elapsed >= 90)  { g_hwNotif.buzzStep = 2; g_hwNotif.buzzStepStartMs = nowMs; } }
    else if (step == 2) { outBuzz = true; if (elapsed >= 220) { g_hwNotif.buzzPat = BUZZ_NONE; g_hwNotif.buzzStep = 0; outBuzz = false; } }
  } else if (pat == BUZZ_NOTIF) {
    outBuzz = true;
    if (elapsed >= 60) { g_hwNotif.buzzPat = BUZZ_NONE; g_hwNotif.buzzStep = 0; outBuzz = false; }
  }
  portEXIT_CRITICAL(&g_hwNotifMux);

  setLedNotifPin(outLed);
  setBuzzerTonePin(outBuzz);
}

// ============================================================================
// SENSOR SIDIK JARI R503 & PENDAFTARAN (ENROLL)
// ============================================================================
int scanFingerprintOnce(uint16_t &outId, int &outConfidence) {
  if (g_r503Mutex && xSemaphoreTake(g_r503Mutex, pdMS_TO_TICKS(150)) != pdTRUE) {
    return -1;
  }

  int result = -1;
  if (!g_r503Ok.load(std::memory_order_acquire)) {
    bool ok = finger.verifyPassword();
    g_r503Ok.store(ok, std::memory_order_release);
    if (!ok) {
      g_lastR503Err.store(FINGERPRINT_PACKETRECIEVEERR, std::memory_order_release);
      if (g_r503Mutex) xSemaphoreGive(g_r503Mutex);
      return -1;
    }
    g_lastR503Err.store(0, std::memory_order_release);
  }

  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
    if (p != FINGERPRINT_NOFINGER) {
      g_lastR503Err.store(p, std::memory_order_release);
      if (p == FINGERPRINT_PACKETRECIEVEERR) {
        g_r503Ok.store(false, std::memory_order_release);
      }
    }
    result = -1;
  } else {
    p = finger.image2Tz(1);
    if (p != FINGERPRINT_OK) {
      g_lastR503Err.store(p, std::memory_order_release);
      result = -2;
    } else {
      p = finger.fingerFastSearch();
      if (p == FINGERPRINT_OK) {
        g_lastR503Err.store(0, std::memory_order_release);
        outId = finger.fingerID;
        outConfidence = static_cast<int>(finger.confidence);
        result = 0;
      } else {
        g_lastR503Err.store(p, std::memory_order_release);
        result = -3;
      }
    }
  }

  if (g_r503Mutex) xSemaphoreGive(g_r503Mutex);
  return result;
}

static bool findFreeFingerprintSlot(uint16_t &outId) {
  for (uint16_t id = 1; id <= FINGER_MAX_SLOTS; id++) {
    uint8_t p = finger.loadModel(id);
    if (p != FINGERPRINT_OK) {
      outId = id;
      return true;
    }
  }
  return false;
}

bool enrollFingerprintNextSlot(uint16_t &enrolledId, char *statusMsg, size_t statusLen) {
  char fallbackBuf[32] = "";
  char *outPtr = (statusMsg && statusLen > 0) ? statusMsg : fallbackBuf;
  size_t outCap = (statusMsg && statusLen > 0) ? statusLen : sizeof(fallbackBuf);

  auto setStat = [&](const char *m) {
    strncpy(outPtr, m, outCap - 1);
    outPtr[outCap - 1] = '\0';
  };

  if (g_r503Mutex && xSemaphoreTake(g_r503Mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    setStat("R503 SIBUK");
    triggerLedAndBuzzer(2, 1000, BUZZ_ALERT);
    return false;
  }

  auto finishEnroll = [&](bool ok, const char *msg, uint8_t ledMode, uint16_t ledDur, BuzzerPattern buzz) -> bool {
    setStat(msg);
    triggerLedAndBuzzer(ledMode, ledDur, buzz);
    if (g_r503Mutex) xSemaphoreGive(g_r503Mutex);
    return ok;
  };

  if (!g_r503Ok.load(std::memory_order_acquire)) {
    bool ok = finger.verifyPassword();
    g_r503Ok.store(ok, std::memory_order_release);
    if (!ok) {
      return finishEnroll(false, "R503 OFFLINE", 2, 1200, BUZZ_ALERT);
    }
  }

  uint16_t slotId = 0;
  if (!findFreeFingerprintSlot(slotId)) {
    return finishEnroll(false, "MEMORI PENUH", 2, 1200, BUZZ_ALERT);
  }

  unsigned long deadline = millis() + 4500;
  setStat("TEMPEL JARI 1/2");
  triggerPopup("ENROLL JARI 1/2", "TEMPELKAN JARI", TFT_YELLOW, 2200);
  triggerLedAndBuzzer(2, 1800, BUZZ_ENROLL_STEP);

  uint8_t p = FINGERPRINT_NOFINGER;
  while (millis() < deadline) {
    esp_task_wdt_reset();
    unsigned long loopNow = millis();
    serviceLedAndBuzzer(isRelayAuthorizedAtomic(), loopNow);
    applyRelayOutput(loopNow);
    p = finger.getImage();
    if (p == FINGERPRINT_OK) break;
    if (p != FINGERPRINT_NOFINGER) {
      return finishEnroll(false, "GAGAL SCAN #1", 2, 1000, BUZZ_ALERT);
    }
    vTaskDelay(pdMS_TO_TICKS(60));
  }
  if (p != FINGERPRINT_OK) {
    return finishEnroll(false, "TIMEOUT JARI 1", 2, 1000, BUZZ_ALERT);
  }

  if (finger.image2Tz(1) != FINGERPRINT_OK) {
    return finishEnroll(false, "CONV GAGAL #1", 2, 1000, BUZZ_ALERT);
  }

  triggerLedAndBuzzer(1, 500, BUZZ_ENROLL_STEP);
  setStat("ANGKAT JARI...");
  triggerPopup("ANGKAT JARI", "TUNGGU SEBENTAR", TFT_CYAN, 2000);
  unsigned long liftDeadline = millis() + 4000;
  while (millis() < liftDeadline && finger.getImage() != FINGERPRINT_NOFINGER) {
    esp_task_wdt_reset();
    serviceLedAndBuzzer(isRelayAuthorizedAtomic(), millis());
    vTaskDelay(pdMS_TO_TICKS(60));
  }

  deadline = millis() + 4500;
  setStat("TEMPEL JARI 2/2");
  triggerPopup("ENROLL JARI 2/2", "TEMPEL LAGI", TFT_YELLOW, 2200);
  triggerLedAndBuzzer(2, 1800, BUZZ_ENROLL_STEP);

  p = FINGERPRINT_NOFINGER;
  while (millis() < deadline) {
    esp_task_wdt_reset();
    unsigned long loopNow = millis();
    serviceLedAndBuzzer(isRelayAuthorizedAtomic(), loopNow);
    applyRelayOutput(loopNow);
    p = finger.getImage();
    if (p == FINGERPRINT_OK) break;
    if (p != FINGERPRINT_NOFINGER) {
      return finishEnroll(false, "GAGAL SCAN #2", 2, 1000, BUZZ_ALERT);
    }
    vTaskDelay(pdMS_TO_TICKS(60));
  }
  if (p != FINGERPRINT_OK) {
    return finishEnroll(false, "TIMEOUT JARI 2", 2, 1000, BUZZ_ALERT);
  }

  if (finger.image2Tz(2) != FINGERPRINT_OK) {
    return finishEnroll(false, "CONV GAGAL #2", 2, 1000, BUZZ_ALERT);
  }

  if (finger.createModel() != FINGERPRINT_OK) {
    return finishEnroll(false, "JARI TDK COCOK", 2, 1200, BUZZ_ALERT);
  }

  if (finger.storeModel(slotId) != FINGERPRINT_OK) {
    return finishEnroll(false, "SIMPAN GAGAL", 2, 1200, BUZZ_ALERT);
  }

  enrolledId = slotId;
  snprintf(outPtr, outCap, "TERDAFTAR #%u", slotId);
  triggerPopup("ENROLL SUKSES", outPtr, TFT_GREEN, 2000);
  triggerLedAndBuzzer(1, 1500, BUZZ_ENROLL_OK);
  if (g_r503Mutex) xSemaphoreGive(g_r503Mutex);
  return true;
}

// ============================================================================
// KOMUNIKASI ESP-NOW TCI (CALLBACK RINGAN -> PROSES DI TASK CORE 0)
// ============================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onTciEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  portENTER_CRITICAL(&g_stateMux);
  g_lastEspSendOk = (status == ESP_NOW_SEND_SUCCESS);
  g_espSendCbSeen = true;
  portEXIT_CRITICAL(&g_stateMux);
}
#else
void onTciEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  portENTER_CRITICAL(&g_stateMux);
  g_lastEspSendOk = (status == ESP_NOW_SEND_SUCCESS);
  g_espSendCbSeen = true;
  portEXIT_CRITICAL(&g_stateMux);
}
#endif

static void syncPendingEspNowPeer() {
  if (!g_tciEspNowReady.load(std::memory_order_acquire)) return;
  bool hasPending = false;
  uint8_t macCopy[6] = {0};

  portENTER_CRITICAL(&g_stateMux);
  if (g_newPeerPending) {
    hasPending = true;
    memcpy(macCopy, g_pendingPeerMac, 6);
    g_newPeerPending = false;
  }
  portEXIT_CRITICAL(&g_stateMux);

  if (hasPending) {
    if (!esp_now_is_peer_exist(macCopy)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, macCopy, 6);
      peer.channel = TCI_WIFI_CHANNEL;
      peer.encrypt = false;
      esp_err_t addErr = esp_now_add_peer(&peer);
      if (addErr != ESP_OK) {
        g_diagTxFail.fetch_add(1, std::memory_order_relaxed);
      }
    }
    saveC3MacToNvs(macCopy);
  }
}

bool sendModePacket(uint8_t mode) {
  if (!isUnlockedAndOnTciPage() || !g_tciEspNowReady.load(std::memory_order_acquire)) {
    return false;
  }

  CommandData cmd;
  cmd.header        = 0xCC55;
  cmd.requestedMode = mode;
  cmd.crc16         = calculateCRC16(reinterpret_cast<const uint8_t*>(&cmd), sizeof(CommandData) - sizeof(uint16_t));

  uint8_t destMac[6];
  portENTER_CRITICAL(&g_stateMux);
  g_espSendCbSeen = false;
  g_lastEspSendOk = false;
  memcpy(destMac, g_c3MacLocked ? activeC3PeerMac : DEFAULT_C3_MAC, 6);
  portEXIT_CRITICAL(&g_stateMux);

  esp_err_t res = esp_now_send(destMac, reinterpret_cast<const uint8_t*>(&cmd), sizeof(CommandData));
  if (res != ESP_OK) {
    g_diagTxFail.fetch_add(1, std::memory_order_relaxed);
    Serial.printf("[ESP-NOW] Gagal kirim mode (%d)\n", static_cast<int>(res));
  }
  unsigned long nowMs = millis();

  portENTER_CRITICAL(&g_stateMux);
  g_lastModeSendMs = nowMs;
  portEXIT_CRITICAL(&g_stateMux);
  return (res == ESP_OK);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onTciEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !info->src_addr || !incomingData) return;
  const uint8_t *mac = info->src_addr;
#else
void onTciEspNowRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
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

static void processIncomingEspNowFrames() {
  if (g_espNowRxQueue == nullptr) return;

  EspNowRawFrame frame;
  while (xQueueReceive(g_espNowRxQueue, &frame, 0) == pdTRUE) {
    if (!isUnlockedAndOnTciPage()) continue;
    if (frame.len != static_cast<int>(sizeof(TelemetryData))) continue;

    const TelemetryData *pkt = reinterpret_cast<const TelemetryData*>(frame.payload);
    if (pkt->header != 0xAA55) continue;

    uint16_t calcCrc = calculateCRC16(frame.payload, sizeof(TelemetryData) - sizeof(uint16_t));
    if (pkt->crc16 != calcCrc) {
      g_diagCrcErr.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    bool peerAccepted = false;
    portENTER_CRITICAL(&g_stateMux);
    if (!STRICT_C3_PEER_LOCK || !g_c3MacLocked) {
      if (memcmp(activeC3PeerMac, frame.srcMac, 6) != 0) {
        if (ALLOW_INITIAL_C3_LEARN || !g_c3MacLocked) {
          memcpy(activeC3PeerMac, frame.srcMac, 6);
          memcpy(g_pendingPeerMac, frame.srcMac, 6);
          g_newPeerPending = true;
          peerAccepted = true;
        }
      } else {
        peerAccepted = true;
      }
      if (STRICT_C3_PEER_LOCK && peerAccepted) {
        g_c3MacLocked = true;
      }
    } else {
      peerAccepted = (memcmp(activeC3PeerMac, frame.srcMac, 6) == 0);
    }
    portEXIT_CRITICAL(&g_stateMux);
    if (!peerAccepted) {
      g_diagMacDrop.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    int degClamped = constrain(static_cast<int>(pkt->degree10), 0, 350);
    uint8_t mapped = static_cast<uint8_t>(map(degClamped, 0, 350, 0, 60));

    portENTER_CRITICAL(&g_tciMux);
    memcpy(&g_tciPkt, pkt, sizeof(TelemetryData));
    g_lastTciRxMs = frame.rxMs;
    g_tciRxSeq++;
    g_advHistory[g_advHistIdx] = mapped;
    g_advHistIdx = (g_advHistIdx + 1) % ADV_HIST_LEN;
    portEXIT_CRITICAL(&g_tciMux);
  }
}

void initTciEspNowOnce() {
  if (g_tciEspNowReady.load(std::memory_order_acquire)) return;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  esp_wifi_set_mac(WIFI_IF_STA, HUD_S3_CUSTOM_MAC);
  esp_wifi_set_channel(TCI_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onTciEspNowRecv);
    esp_now_register_send_cb(onTciEspNowSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, DEFAULT_C3_MAC, 6);
    peer.channel = TCI_WIFI_CHANNEL;
    peer.encrypt = false;
    bool basePeerOk = (esp_now_add_peer(&peer) == ESP_OK);

    uint8_t savedMac[6];
    bool hasLocked = false;
    portENTER_CRITICAL(&g_stateMux);
    hasLocked = g_c3MacLocked;
    memcpy(savedMac, activeC3PeerMac, 6);
    portEXIT_CRITICAL(&g_stateMux);

    if (hasLocked && memcmp(savedMac, DEFAULT_C3_MAC, 6) != 0 && !esp_now_is_peer_exist(savedMac)) {
      esp_now_peer_info_t savedPeer = {};
      memcpy(savedPeer.peer_addr, savedMac, 6);
      savedPeer.channel = TCI_WIFI_CHANNEL;
      savedPeer.encrypt = false;
      esp_now_add_peer(&savedPeer);
    }

    if (basePeerOk) {
      g_tciEspNowReady.store(true, std::memory_order_release);
    } else {
      esp_now_deinit();
    }
  }
}

void setTciGateForPage(bool onTciPage) {
  if (!onTciPage) {
    portENTER_CRITICAL(&g_stateMux);
    g_pendingModeChange = false;
    g_newPeerPending    = false;
    portEXIT_CRITICAL(&g_stateMux);
  }
  g_tciRxEnabled.store(onTciPage, std::memory_order_release);
  g_allowChronosNotif.store(!onTciPage, std::memory_order_release);
}

// ============================================================================
// PARSER NAVIGASI & NOTIFIKASI CHRONOS
// ============================================================================
static NavTurnType classifyNavTurnFromText(const char *dirText, const char *titleText) {
  const char *parts[2] = {dirText ? dirText : "", titleText ? titleText : ""};
  for (int p = 0; p < 2; p++) {
    const char *s = parts[p];
    if (containsIgnoreCase(s, "arrive") || containsIgnoreCase(s, "destination") ||
        containsIgnoreCase(s, "tiba") || containsIgnoreCase(s, "tujuan") || containsIgnoreCase(s, "sampai")) {
      return NAV_TURN_ARRIVE;
    }
    if (containsIgnoreCase(s, "u-turn") || containsIgnoreCase(s, "uturn") || containsIgnoreCase(s, "u turn") ||
        containsIgnoreCase(s, "putar balik") || containsIgnoreCase(s, "balik arah")) {
      return NAV_TURN_U_TURN;
    }
    if (containsIgnoreCase(s, "roundabout") || containsIgnoreCase(s, "rotary") ||
        containsIgnoreCase(s, "bundaran")) {
      return NAV_TURN_ROUNDABOUT;
    }
    if (containsIgnoreCase(s, "slight left") || containsIgnoreCase(s, "keep left") ||
        containsIgnoreCase(s, "serong kiri") || containsIgnoreCase(s, "ambil kiri") || containsIgnoreCase(s, "tetap di kiri")) {
      return NAV_TURN_SLIGHT_LEFT;
    }
    if (containsIgnoreCase(s, "slight right") || containsIgnoreCase(s, "keep right") ||
        containsIgnoreCase(s, "serong kanan") || containsIgnoreCase(s, "ambil kanan") || containsIgnoreCase(s, "tetap di kanan")) {
      return NAV_TURN_SLIGHT_RIGHT;
    }
    if (containsIgnoreCase(s, "left") || containsIgnoreCase(s, "kiri")) {
      return NAV_TURN_LEFT;
    }
    if (containsIgnoreCase(s, "right") || containsIgnoreCase(s, "kanan")) {
      return NAV_TURN_RIGHT;
    }
  }
  return NAV_TURN_STRAIGHT;
}

static bool isDistanceToken(const char *s, char *outBuf, size_t outSize) {
  if (!s || s[0] == '\0') return false;
  size_t len = strlen(s);
  for (size_t i = 0; i < len; i++) {
    if (isdigit(static_cast<unsigned char>(s[i]))) {
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
  double val = 0.0;
  bool hasVal = false;
  char numBuf[16];
  size_t nIdx = 0;
  for (size_t i = 0; distText[i] != '\0' && nIdx < sizeof(numBuf) - 1; i++) {
    char c = distText[i];
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
  val = atof(numBuf);
  if (val < 0.0) return 999999999UL;

  bool isKm = containsIgnoreCase(distText, "km");
  if (isKm) {
    return static_cast<uint32_t>(val * 1000.0);
  }
  return static_cast<uint32_t>(val);
}

static const char* classifyProximityCue(uint32_t distMeters, NavTurnType turn) {
  if (turn == NAV_TURN_ARRIVE) {
    if (distMeters <= 40) return "TIBA DI TUJUAN";
    return "MENUJU TUJUAN";
  }
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

static void extractSpeedInfo(const char *speedSrc, const char *titleSrc, const char *dirSrc, char *outBuf, size_t outSize) {
  const char *cands[3] = {speedSrc, titleSrc, dirSrc};
  for (int i = 0; i < 3; i++) {
    const char *c = cands[i];
    if (!c || c[0] == '\0') continue;

    const char *units[3] = {"km/h", "km/j", "kpj"};
    for (int u = 0; u < 3; u++) {
      const char *p = c;
      while (*p != '\0') {
        const char *q = p;
        size_t uLen = strlen(units[u]);
        bool match = true;
        for (size_t k = 0; k < uLen; k++) {
          if (q[k] == '\0' || tolower(static_cast<unsigned char>(q[k])) != units[u][k]) {
            match = false;
            break;
          }
        }
        if (match && q > c) {
          const char *endNum = q - 1;
          while (endNum >= c && *endNum == ' ') endNum--;
          const char *startNum = endNum;
          bool hasDigit = false;
          while (startNum >= c && (isdigit(static_cast<unsigned char>(*startNum)) || *startNum == '.' || *startNum == ',')) {
            if (isdigit(static_cast<unsigned char>(*startNum))) hasDigit = true;
            startNum--;
          }
          startNum++;
          while (startNum <= endNum && (*startNum == '.' || *startNum == ',')) startNum++;
          while (endNum >= startNum && (*endNum == '.' || *endNum == ',')) endNum--;

          if (hasDigit && endNum >= startNum) {
            size_t nLen = static_cast<size_t>(endNum - startNum + 1);
            char numBuf[10];
            if (nLen >= sizeof(numBuf)) nLen = sizeof(numBuf) - 1;
            memcpy(numBuf, startNum, nLen);
            numBuf[nLen] = '\0';
            for (size_t m = 0; m < nLen; m++) {
              if (numBuf[m] == ',') numBuf[m] = '.';
            }
            snprintf(outBuf, outSize, "%s KM/J", numBuf);
            return;
          }
        }
        p++;
      }
    }

    if (i == 0) {
      bool validNum = true;
      bool hasDigit = false;
      for (size_t k = 0; c[k] != '\0'; k++) {
        unsigned char ch = static_cast<unsigned char>(c[k]);
        if (isdigit(ch)) {
          hasDigit = true;
        } else if (ch != '.' && ch != ',') {
          validNum = false;
          break;
        }
      }
      if (validNum && hasDigit) {
        snprintf(outBuf, outSize, "%s KM/J", c);
        return;
      }
    }
  }
  snprintf(outBuf, outSize, "-- KM/J");
}

void updateNavFromChronos() {
  bool bleConn = watch.isConnected();
  Navigation nav = watch.getNavigation();
  unsigned long nowMs = millis();

  char dirBuf[48]   = "";
  char titleBuf[48] = "";
  char distBuf[20]  = "";
  char etaBuf[28]   = "";
  char durBuf[22]   = "";
  char spdRaw[20]   = "";

  if (bleConn && nav.active) {
    strncpy(dirBuf, nav.directions.c_str(), sizeof(dirBuf) - 1);
    dirBuf[sizeof(dirBuf) - 1] = '\0';
    sanitizeAsciiInPlace(dirBuf);
    trimAsciiInPlace(dirBuf);

    strncpy(titleBuf, nav.title.c_str(), sizeof(titleBuf) - 1);
    titleBuf[sizeof(titleBuf) - 1] = '\0';
    sanitizeAsciiInPlace(titleBuf);
    trimAsciiInPlace(titleBuf);

    strncpy(distBuf, nav.distance.c_str(), sizeof(distBuf) - 1);
    distBuf[sizeof(distBuf) - 1] = '\0';
    sanitizeAsciiInPlace(distBuf);
    trimAsciiInPlace(distBuf);

    strncpy(etaBuf, nav.eta.c_str(), sizeof(etaBuf) - 1);
    etaBuf[sizeof(etaBuf) - 1] = '\0';
    sanitizeAsciiInPlace(etaBuf);
    trimAsciiInPlace(etaBuf);

    strncpy(durBuf, nav.duration.c_str(), sizeof(durBuf) - 1);
    durBuf[sizeof(durBuf) - 1] = '\0';
    sanitizeAsciiInPlace(durBuf);
    trimAsciiInPlace(durBuf);

    strncpy(spdRaw, nav.speed.c_str(), sizeof(spdRaw) - 1);
    spdRaw[sizeof(spdRaw) - 1] = '\0';
    sanitizeAsciiInPlace(spdRaw);
    trimAsciiInPlace(spdRaw);
  }

  NavTelemetryData nextNav;
  memset(&nextNav, 0, sizeof(nextNav));
  nextNav.bleConnected = bleConn;

  if (bleConn && nav.active) {
    NavTurnType tType = classifyNavTurnFromText(dirBuf, titleBuf);
    nextNav.navActive = true;
    nextNav.turn      = tType;

    strncpy(nextNav.turnLabel, navTurnDefaultLabel(tType), sizeof(nextNav.turnLabel) - 1);
    nextNav.turnLabel[sizeof(nextNav.turnLabel) - 1] = '\0';

    char parsedDist[16] = "";
    const char* distSrc = "--";
    if (distBuf[0] != '\0') {
      if (isDistanceToken(distBuf, parsedDist, sizeof(parsedDist))) {
        distSrc = parsedDist;
      } else {
        distSrc = distBuf;
      }
    } else if (isDistanceToken(dirBuf, parsedDist, sizeof(parsedDist)) ||
               isDistanceToken(titleBuf, parsedDist, sizeof(parsedDist))) {
      distSrc = parsedDist;
    }
    strncpy(nextNav.distance, distSrc, sizeof(nextNav.distance) - 1);
    nextNav.distance[sizeof(nextNav.distance) - 1] = '\0';

    const char* roadSrc = (dirBuf[0] != '\0') ? dirBuf
                        : ((titleBuf[0] != '\0') ? titleBuf : "Ikuti Rute GMaps");
    strncpy(nextNav.roadName, roadSrc, sizeof(nextNav.roadName) - 1);
    nextNav.roadName[sizeof(nextNav.roadName) - 1] = '\0';

    const char* etaSrc = (etaBuf[0] != '\0') ? etaBuf : "--";
    strncpy(nextNav.etaInfo, etaSrc, sizeof(nextNav.etaInfo) - 1);
    nextNav.etaInfo[sizeof(nextNav.etaInfo) - 1] = '\0';

    const char* durSrc = (durBuf[0] != '\0') ? durBuf : "--";
    strncpy(nextNav.durationInfo, durSrc, sizeof(nextNav.durationInfo) - 1);
    nextNav.durationInfo[sizeof(nextNav.durationInfo) - 1] = '\0';

    extractSpeedInfo(spdRaw, titleBuf, dirBuf, nextNav.speedInfo, sizeof(nextNav.speedInfo));
    nextNav.lastRxMs = nowMs;
  }

  portENTER_CRITICAL(&g_navMux);
  uint32_t prevSeq = g_navData.seq;
  bool prevBle = g_navData.bleConnected;
  bool prevAct = g_navData.navActive;
  if (bleConn && nav.active) {
    nextNav.seq = prevSeq + 1;
    g_navData = nextNav;
  } else {
    g_navData.bleConnected = bleConn;
    g_navData.navActive = false;
    if (prevBle != bleConn || prevAct != false) {
      g_navData.seq = prevSeq + 1;
    }
  }
  portEXIT_CRITICAL(&g_navMux);
}

void pollChronosNotification() {
  if (!g_allowChronosNotif.load(std::memory_order_acquire)) return;

  static char lastSig[164] = "";
  static bool wasBleConn = false;

  bool bleConn = watch.isConnected();
  if (!bleConn) {
    if (wasBleConn) {
      lastSig[0] = '\0';
      wasBleConn = false;
    }
    return;
  }
  wasBleConn = true;

  Notification n = watch.getNotificationAt(0);
  char appBuf[24];
  char titleBuf[32];
  char msgBuf[104];

  strncpy(appBuf, n.app.c_str(), sizeof(appBuf) - 1);
  appBuf[sizeof(appBuf) - 1] = '\0';
  sanitizeAsciiInPlace(appBuf);
  trimAsciiInPlace(appBuf);

  strncpy(titleBuf, n.title.c_str(), sizeof(titleBuf) - 1);
  titleBuf[sizeof(titleBuf) - 1] = '\0';
  sanitizeAsciiInPlace(titleBuf);
  trimAsciiInPlace(titleBuf);

  strncpy(msgBuf, n.message.c_str(), sizeof(msgBuf) - 1);
  msgBuf[sizeof(msgBuf) - 1] = '\0';
  sanitizeAsciiInPlace(msgBuf, false);
  trimAsciiInPlace(msgBuf);

  if (appBuf[0] == '\0' && titleBuf[0] == '\0' && msgBuf[0] == '\0') return;

  char sig[164];
  snprintf(sig, sizeof(sig), "%s|%s|%s", appBuf, titleBuf, msgBuf);
  if (strcmp(sig, lastSig) == 0) return;
  memcpy(lastSig, sig, sizeof(lastSig));

  unsigned long nowMs = millis();
  NotifPopupData nextNotif;
  memset(&nextNotif, 0, sizeof(nextNotif));
  strncpy(nextNotif.app, appBuf[0] != '\0' ? appBuf : "NOTIF", sizeof(nextNotif.app) - 1);
  nextNotif.app[sizeof(nextNotif.app) - 1] = '\0';

  strncpy(nextNotif.title, titleBuf, sizeof(nextNotif.title) - 1);
  nextNotif.title[sizeof(nextNotif.title) - 1] = '\0';

  strncpy(nextNotif.message, msgBuf, sizeof(nextNotif.message) - 1);
  nextNotif.message[sizeof(nextNotif.message) - 1] = '\0';

  nextNotif.startMs    = nowMs;
  nextNotif.durationMs = NOTIF_POPUP_MS;
  nextNotif.active     = true;

  portENTER_CRITICAL(&g_notifMux);
  nextNotif.seq = g_notifPopup.seq + 1;
  g_notifPopup = nextNotif;
  portEXIT_CRITICAL(&g_notifMux);

  triggerLedAndBuzzer(2, 1500, BUZZ_NOTIF, nowMs);
}

// ============================================================================
// TRANSISI & RENDERING UI LCD (RESPONSIF UNTUK BERKENDARA HARIAN)
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
};

static UiTransitionState g_uiTrans = {false, UI_TRANS_NONE, 0, 0, 8, false};

static void startComponentTransition(uint16_t stepDelayMs = 8) {
  g_uiTrans.active      = true;
  g_uiTrans.mode        = UI_TRANS_EXIT_ENTER;
  g_uiTrans.step        = 0;
  g_uiTrans.lastStepMs  = millis();
  g_uiTrans.stepDelayMs = stepDelayMs;
  g_uiTrans.exitDone    = false;
}

static void drawAccentSweep(int y, uint16_t color, int progress0To320) {
  if (progress0To320 <= 0) return;
  int w = constrain(progress0To320, 0, 320);
  lcd.fillRect(0, y, w, 2, color);
}

static void animateExitComponents(uint8_t exitPhase) {
  if (exitPhase == 0) {
    lcd.fillRect(0, 0, 320, 120, TFT_BLACK);
    drawAccentSweep(120, TFT_CYAN, 200);
  } else {
    lcd.fillRect(0, 120, 320, 120, TFT_BLACK);
  }
}

static void animateEnterComponentStage(uint8_t enterStage) {
  if (enterStage == 0) {
    lcd.drawFastHLine(0, 26, 320, TFT_CYAN);
  } else if (enterStage == 1) {
    lcd.drawFastHLine(8, 112, 304, 0x2104);
  } else {
    lcd.drawFastHLine(0, 206, 320, TFT_DARKGREY);
  }
}

void drawTopBar(const char* title, bool relayOn, bool emgBypass = false) {
  lcd.fillRect(0, 0, 320, 26, TFT_NAVY);
  lcd.drawFastHLine(0, 26, 320, TFT_CYAN);
  lcd.setTextColor(TFT_WHITE, TFT_NAVY);
  lcd.setTextSize(2);
  lcd.setCursor(10, 5);
  lcd.print(title);

  lcd.setTextSize(1);
  if (emgBypass) {
    lcd.setTextColor(TFT_ORANGE, TFT_NAVY);
    lcd.setCursor(225, 9);
    lcd.print("EMG: BYPASS");
  } else {
    lcd.setTextColor(relayOn ? TFT_GREEN : TFT_RED, TFT_NAVY);
    lcd.setCursor(235, 9);
    lcd.print(relayOn ? "RELAY: ON " : "RELAY: OFF");
  }
}

void drawTciPopup(const char* line1, const char* line2, uint16_t bgColor) {
  lcd.fillRect(60, 75, 200, 82, bgColor);
  lcd.drawRect(64, 79, 192, 74, TFT_BLACK);
  lcd.setTextColor(TFT_BLACK, bgColor);
  lcd.setTextSize(2);
  lcd.setCursor(76, 88);
  lcd.print(line1);
  lcd.setCursor(76, 118);
  lcd.print(line2);
}

void drawNotifPopup(const NotifPopupData &n) {
  const int boxX = 14, boxY = 34, boxW = 292, boxH = 174;
  uint16_t hdrCol = TFT_NAVY;
  if (containsIgnoreCase(n.app, "whatsapp") || containsIgnoreCase(n.app, "wa")) {
    hdrCol = TFT_DARKGREEN;
  }

  lcd.fillRoundRect(boxX, boxY, boxW, boxH, 6, TFT_BLACK);
  lcd.drawRoundRect(boxX, boxY, boxW, boxH, 6, TFT_CYAN);
  lcd.fillRect(boxX + 2, boxY + 2, boxW - 4, 24, hdrCol);

  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, hdrCol);
  lcd.setCursor(boxX + 10, boxY + 10);
  lcd.printf("APP: %-24s", n.app);

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.setCursor(boxX + 10, boxY + 34);
  char tClip[22];
  strncpy(tClip, n.title, 21);
  tClip[21] = '\0';
  lcd.printf("%-21s", tClip);

  lcd.drawFastHLine(boxX + 8, boxY + 56, boxW - 16, TFT_DARKGREY);

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  const int maxCharsPerLine = 21;
  int msgLen = static_cast<int>(strlen(n.message));
  int pos = 0;

  for (int row = 0; row < 3; row++) {
    while (pos < msgLen && (n.message[pos] == ' ' || n.message[pos] == '\r')) {
      pos++;
    }
    if (pos < msgLen && n.message[pos] == '\n') {
      pos++;
    }

    char line[22];
    memset(line, ' ', 21);
    line[21] = '\0';

    int len = 0;
    int lastSpace = -1;
    int scanPos = pos;

    while (scanPos < msgLen && len < maxCharsPerLine) {
      char ch = n.message[scanPos];
      if (ch == '\n' || ch == '\r') break;
      if (ch == ' ') lastSpace = len;
      line[len++] = ch;
      scanPos++;
    }

    int lineChars = len;
    if (scanPos < msgLen && n.message[scanPos] != '\n' && n.message[scanPos] != '\r' && lastSpace > 0) {
      lineChars = lastSpace;
      pos += lastSpace + 1;
    } else {
      pos = scanPos;
      if (pos < msgLen && n.message[pos] == '\n') pos++;
    }

    for (int k = lineChars; k < 21; k++) {
      line[k] = ' ';
    }

    lcd.setCursor(boxX + 10, boxY + 66 + (row * 28));
    lcd.print(line);
  }
}

void renderBootLockScreen(bool fullClear, bool emgBypass, bool r503Ok, uint8_t lastR503Err, bool rtcOk, bool rtcLostPower, uint8_t enterStage = 255) {
  if (fullClear) {
    lcd.fillScreen(TFT_BLACK);
  }

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.drawRoundRect(4, 4, 312, 232, 6, TFT_CYAN);
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(18, 26);
    lcd.print("IMMOBILIZER R503");
    lcd.drawFastHLine(18, 50, 284, TFT_DARKGREY);
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.setCursor(18, 66);
    lcd.print("Tempelkan jari pada R503");
    lcd.setCursor(18, 94);
    lcd.print("untuk ON Relay & Sistem ");
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(1);
    lcd.setTextColor(r503Ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    lcd.setCursor(18, 144);
    if (r503Ok) {
      lcd.print("Status R503: TERHUBUNG            ");
    } else {
      lcd.printf("Status R503: ERR (0x%02X) CEK UART  ", lastR503Err);
    }

    lcd.setTextColor(rtcOk ? (rtcLostPower ? TFT_YELLOW : TFT_GREEN) : TFT_RED, TFT_BLACK);
    lcd.setCursor(18, 162);
    if (!rtcOk) {
      lcd.print("Status RTC : TIDAK TERDETEKSI     ");
    } else if (rtcLostPower) {
      lcd.print("Status RTC : OK (BATERAI LEMAH)   ");
    } else {
      lcd.print("Status RTC : TERHUBUNG            ");
    }

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
    lcd.setCursor(18, 188);
    lcd.print("Emergency (GPIO2): Tahan 3.5s ON Relay");

    lcd.setTextColor(emgBypass ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(18, 208);
    lcd.print(emgBypass ? "RELAY: ON (EMERGENCY BYPASS AKTIF)  "
                        : "RELAY: OFF (CORE 1 SECURITY ACTIVE) ");
  }
}

void renderPageHome(bool fullClear, bool relayOn, bool emgBypass, bool rtcOk, bool rtcLostPower, const char* tBuf, const char* dBuf, const char* fBuf, float chipTempC, uint8_t enterStage = 255) {
  if (fullClear) {
    lcd.fillScreen(TFT_BLACK);
    drawTopBar("1. HOME", relayOn, emgBypass);
  }

  if (fullClear || enterStage == 0 || enterStage == 255) {
    drawTopBar("1. HOME", relayOn, emgBypass);
    lcd.drawRoundRect(8, 32, 304, 72, 5, 0x2104);
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.drawRoundRect(8, 108, 304, 64, 5, 0x2104);
    lcd.setTextSize(2);
    lcd.setTextColor(emgBypass ? TFT_ORANGE : TFT_GREEN, TFT_BLACK);
    lcd.setCursor(14, 114);
    lcd.printf("ID JARI: %-14s", fBuf);

    lcd.setTextColor(rtcOk ? (rtcLostPower ? TFT_ORANGE : TFT_YELLOW) : TFT_RED, TFT_BLACK);
    lcd.setCursor(14, 144);
    if (!rtcOk) {
      lcd.print("RTC: NO RTC           ");
    } else if (rtcLostPower) {
      lcd.print("RTC: SET ULANG JAM    ");
    } else {
      lcd.print("RTC: DS3231 OK        ");
    }
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(14, 208);
    lcd.print("1x Tekan: Halaman Berikutnya (1 -> 2 -> 3)");
    lcd.setCursor(14, 222);
    lcd.print("Tahan 3s: Buka Jok (Lepas = Relay OFF)");
  }

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.setTextSize(3);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(14, 38);
    lcd.printf("%-8s", tBuf);

    lcd.setTextSize(2);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(14, 74);
    lcd.printf("%-12s", dBuf);

    uint16_t tempCol = (chipTempC >= 75.0f) ? TFT_RED : ((chipTempC >= 55.0f) ? TFT_ORANGE : TFT_CYAN);
    lcd.setTextSize(2);
    lcd.setTextColor(tempCol, TFT_BLACK);
    lcd.setCursor(192, 42);
    lcd.printf("MCU:%4.1fC", chipTempC);
  }
}

void drawNavTurnIcon(int cx, int cy, NavTurnType turn, uint16_t col) {
  lcd.fillRect(cx - 34, cy - 34, 68, 68, TFT_BLACK);
  lcd.drawRoundRect(cx - 34, cy - 34, 68, 68, 8, TFT_DARKGREY);

  switch (turn) {
    case NAV_TURN_LEFT:
      lcd.fillRect(cx - 6, cy - 6, 18, 22, col);
      lcd.fillRect(cx - 12, cy - 10, 24, 8, col);
      lcd.fillTriangle(cx - 24, cy - 6, cx - 10, cy - 18, cx - 10, cy + 6, col);
      break;
    case NAV_TURN_RIGHT:
      lcd.fillRect(cx - 12, cy - 6, 18, 22, col);
      lcd.fillRect(cx - 12, cy - 10, 24, 8, col);
      lcd.fillTriangle(cx + 24, cy - 6, cx + 10, cy - 18, cx + 10, cy + 6, col);
      break;
    case NAV_TURN_SLIGHT_LEFT:
      lcd.drawLine(cx + 10, cy + 18, cx - 14, cy - 14, col);
      lcd.fillTriangle(cx - 20, cy - 20, cx - 6, cy - 16, cx - 16, cy - 6, col);
      break;
    case NAV_TURN_SLIGHT_RIGHT:
      lcd.drawLine(cx - 10, cy + 18, cx + 14, cy - 14, col);
      lcd.fillTriangle(cx + 20, cy - 20, cx + 6, cy - 16, cx + 16, cy - 6, col);
      break;
    case NAV_TURN_U_TURN:
      lcd.drawArc(cx + 4, cy + 2, 14, 14, 180, 360, col);
      lcd.drawFastVLine(cx + 18, cy + 2, 16, col);
      lcd.drawFastVLine(cx - 10, cy + 2, 10, col);
      lcd.fillTriangle(cx - 10, cy + 22, cx - 17, cy + 12, cx - 3, cy + 12, col);
      break;
    case NAV_TURN_ROUNDABOUT:
      lcd.drawCircle(cx, cy + 2, 14, col);
      lcd.fillTriangle(cx, cy - 22, cx - 8, cy - 10, cx + 8, cy - 10, col);
      break;
    case NAV_TURN_ARRIVE:
      lcd.fillCircle(cx, cy, 14, col);
      lcd.fillCircle(cx, cy, 5, TFT_BLACK);
      break;
    case NAV_TURN_STRAIGHT:
    default:
      lcd.fillRect(cx - 5, cy - 6, 10, 26, col);
      lcd.fillTriangle(cx, cy - 22, cx - 14, cy - 4, cx + 14, cy - 4, col);
      break;
  }
}

void renderPageNav(bool fullClear, bool relayOn, bool emgBypass, const NavTelemetryData &nav, bool navOnline, uint8_t enterStage = 255) {
  if (fullClear) {
    lcd.fillScreen(TFT_BLACK);
    drawTopBar("2. NAVIGASI", relayOn, emgBypass);
  }

  if (fullClear || enterStage == 0 || enterStage == 255) {
    drawTopBar("2. NAVIGASI", relayOn, emgBypass);

    lcd.setTextSize(1);
    lcd.setCursor(12, 32);
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

    uint16_t turnCol = navOnline ? TFT_CYAN : TFT_DARKGREY;
    drawNavTurnIcon(48, 76, navOnline ? nav.turn : NAV_TURN_NONE, turnCol);

    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_WHITE : TFT_SILVER, TFT_BLACK);
    lcd.setCursor(92, 46);
    lcd.printf("%-13s", navOnline ? nav.turnLabel : "NO NAV");

    lcd.setTextSize(3);
    lcd.setTextColor(navOnline ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(92, 72);
    lcd.printf("%-10s", navOnline ? nav.distance : "--");
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    uint32_t distM = navOnline ? parseDistanceMeters(nav.distance) : 999999999UL;
    const char *cue = navOnline ? classifyProximityCue(distM, nav.turn) : "MENUNGGU RUTE";
    uint16_t cueCol = TFT_DARKGREY;
    if (navOnline) {
      if (distM <= 35) cueCol = TFT_RED;
      else if (distM <= 120) cueCol = TFT_ORANGE;
      else if (distM <= 350) cueCol = TFT_YELLOW;
      else cueCol = TFT_CYAN;
    }

    lcd.fillRect(10, 114, 300, 34, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setTextColor(cueCol, TFT_BLACK);
    lcd.setCursor(12, 115);
    lcd.print(cue);

    char roadClip[26];
    if (navOnline) {
      strncpy(roadClip, nav.roadName, 25);
      roadClip[25] = '\0';
    } else if (nav.bleConnected) {
      strncpy(roadClip, "Buka rute di Google Maps", 25);
      roadClip[25] = '\0';
    } else {
      strncpy(roadClip, "Hubungkan HP di Chronos", 25);
      roadClip[25] = '\0';
    }
    lcd.setTextSize(1);
    lcd.setTextColor(navOnline ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(12, 136);
    lcd.print(roadClip);
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.drawFastHLine(0, 150, 320, TFT_DARKGREY);
    lcd.drawFastVLine(108, 150, 56, TFT_DARKGREY);
    lcd.drawFastVLine(216, 150, 56, TFT_DARKGREY);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(8, 158);
    lcd.print("KECEPATAN");

    char spdClip[9];
    strncpy(spdClip, navOnline ? nav.speedInfo : "-- KM/J", 8);
    spdClip[8] = '\0';
    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(8, 176);
    lcd.printf("%-8s", spdClip);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(115, 158);
    lcd.print("SISA WAKTU");

    char durClip[9];
    strncpy(durClip, navOnline ? nav.durationInfo : "--", 8);
    durClip[8] = '\0';
    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_CYAN : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(115, 176);
    lcd.printf("%-8s", durClip);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.setCursor(223, 158);
    lcd.print("EST. TIBA");

    char etaClip[9];
    strncpy(etaClip, navOnline ? nav.etaInfo : "--", 8);
    etaClip[8] = '\0';
    lcd.setTextSize(2);
    lcd.setTextColor(navOnline ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(223, 176);
    lcd.printf("%-8s", etaClip);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(12, 212);
    lcd.print("1x: Masuk TCI | Tahan 3s: Buka Jok");
    lcd.setCursor(12, 225);
    lcd.printf("BLE Chronos: %-20s", BLE_DEVICE_NAME);
  }
}

void renderPageTciDashboard(bool fullClear, bool relayOn, bool emgBypass, const TelemetryData &d, bool online, uint8_t enterStage = 255) {
  if (fullClear) {
    lcd.fillScreen(TFT_BLACK);
  }
  if (fullClear || enterStage == 0 || enterStage == 255) {
    drawTopBar("3. TCI DASHBOARD", relayOn, emgBypass);
  }

  if (!online) {
    if (fullClear || enterStage >= 1 || enterStage == 255) {
      lcd.drawRoundRect(10, 34, 300, 178, 6, TFT_RED);
      lcd.fillRect(11, 35, 298, 176, TFT_BLACK);
      lcd.setTextColor(TFT_RED, TFT_BLACK);
      lcd.setTextSize(3);
      lcd.setCursor(65, 75);
      lcd.print("NO SIGNAL");
      lcd.setTextSize(2);
      lcd.setCursor(55, 125);
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
    lcd.setCursor(12, 34);
    lcd.printf("MODE:%-4s", modeStr);

    uint16_t voltCol = (d.battVolt10 < BATT_MIN_WARN_10 || d.battVolt10 > BATT_MAX_WARN_10) ? TFT_RED : TFT_WHITE;
    lcd.setTextColor(voltCol, TFT_BLACK);
    lcd.setCursor(190, 34);
    lcd.printf("%4.1fV", d.battVolt10 / 10.0f);

    lcd.setTextSize(3);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(12, 62);
    lcd.printf("RPM:%5u", d.rpm);

    int barW = map(constrain(static_cast<int>(d.rpm), 0, static_cast<int>(MAX_RPM_SCALE)),
                   0, static_cast<int>(MAX_RPM_SCALE), 0, 294);
    lcd.drawRect(12, 96, 296, 14, TFT_DARKGREY);
    uint16_t barCol = (d.rpm > RPM_WARN_RED) ? TFT_RED : ((d.rpm > RPM_WARN_YELLOW) ? TFT_YELLOW : TFT_GREEN);
    if (barW > 0) {
      lcd.fillRect(13, 97, barW, 12, barCol);
    }
    if (barW < 294) {
      lcd.fillRect(13 + barW, 97, 294 - barW, 12, TFT_BLACK);
    }
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(12, 120);
    lcd.printf("Adv :%5.1f*", d.degree10 / 10.0f);

    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(175, 120);
    lcd.printf("TPS:%3u%%", d.tps);

    uint16_t tempCol = (d.temp10 >= ENG_TEMP_WARN_10) ? TFT_RED : ((d.temp10 > 900) ? TFT_YELLOW : TFT_ORANGE);
    lcd.setTextColor(tempCol, TFT_BLACK);
    lcd.setCursor(12, 148);
    lcd.printf("Temp:%5.0fC", d.temp10 / 10.0f);
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setCursor(12, 178);
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
    lcd.setCursor(12, 222);
    lcd.print("1x: Kurva | 2x: Home | Tahan 3s: Mode (RPM<=400)");
  }
}

void renderPageTciCurve(bool fullClear, bool relayOn, bool emgBypass, const TelemetryData &d, const uint8_t *hist, int histIdx, bool online, uint8_t enterStage = 255) {
  if (fullClear) {
    lcd.fillScreen(TFT_BLACK);
  }
  if (fullClear || enterStage == 0 || enterStage == 255) {
    drawTopBar("3. ADV CURVE", relayOn, emgBypass);
  }

  constexpr int plotX = 16;
  constexpr int plotY = 36;
  constexpr int plotW = ADV_HIST_LEN;
  constexpr int plotH = 148;
  constexpr int baseY = plotY + plotH - 2;

  if (fullClear || enterStage == 0 || enterStage == 255) {
    lcd.fillRect(plotX + 1, plotY + 1, plotW - 2, plotH - 2, TFT_BLACK);
    lcd.drawRoundRect(plotX, plotY, plotW, plotH, 4, TFT_DARKGREY);

    if (!online) {
      lcd.setTextColor(TFT_RED, TFT_BLACK);
      lcd.setTextSize(2);
      lcd.setCursor(38, 100);
      lcd.print("NO SIGNAL");
      return;
    }

    lcd.drawFastHLine(plotX + 1, plotY + plotH / 2, plotW - 2, 0x2104);

    int prevX = plotX + 1;
    int prevY = baseY;
    for (int x = 0; x < ADV_HIST_LEN - 2; x++) {
      int idx = (histIdx + x) % ADV_HIST_LEN;
      int scaledY = map(hist[idx], 0, 60, 0, plotH - 6);
      int curY = baseY - scaledY;
      if (x > 0) {
        lcd.drawLine(prevX, prevY, plotX + 1 + x, curY, TFT_GREEN);
      }
      prevX = plotX + 1 + x;
      prevY = curY;
    }
  }

  if (fullClear || enterStage == 1 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(186, 44);
    lcd.printf("ADV:%4.1f*", d.degree10 / 10.0f);

    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(186, 76);
    lcd.printf("M:%u", d.mode);
  }

  if (fullClear || enterStage >= 2 || enterStage == 255) {
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(14, 196);
    lcd.printf("RPM:%5u  TPS:%3u%%", d.rpm, d.tps);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(14, 222);
    lcd.print("1x: Dashboard | 2x: Home | Tahan 3s: Mode");
  }
}

// ============================================================================
// FREERTOS TASKS
// ============================================================================
void taskImmobilizerCore1(void* pvParameters) {
  (void)pvParameters;
  esp_task_wdt_add(nullptr);

  bool lastReading             = HIGH;
  bool stableState             = HIGH;
  bool isPressed               = false;
  bool longPressSent           = false;
  uint8_t pendingClicks        = 0;
  unsigned long lastDebMs      = 0;
  unsigned long pressStartMs   = 0;
  unsigned long lastClickRelMs = 0;
  unsigned long lastFingerMs   = 0;

  bool emLastReading    = HIGH;
  bool emStableState    = HIGH;
  bool emPressed        = false;
  bool emTriggered      = false;
  unsigned long emDebMs = 0;
  unsigned long emStart = 0;

  bool enLastReading    = HIGH;
  bool enStableState    = HIGH;
  bool enPressed        = false;
  bool enTriggered      = false;
  unsigned long enDebMs = 0;
  unsigned long enStart = 0;

  auto resetMainButtonState = [&]() {
    lastReading   = HIGH;
    stableState   = HIGH;
    isPressed     = false;
    longPressSent = false;
    pendingClicks = 0;
  };

  for (;;) {
    esp_task_wdt_reset();
    unsigned long nowMs = millis();
    serviceLedAndBuzzer(g_systemUnlocked || g_emergencyByPass, nowMs);

    // --- 0. Tombol Pendaftaran Sidik Jari R503 (GPIO 38, tahan 2 detik) ---
    bool enReading = digitalRead(PIN_BTN_ENROLL);
    if (enReading != enLastReading) enDebMs = nowMs;
    if ((nowMs - enDebMs) > DEBOUNCE_MS) {
      if (enReading != enStableState) {
        enStableState = enReading;
        if (enStableState == LOW) {
          enPressed   = true;
          enTriggered = false;
          enStart     = nowMs;
        } else {
          enPressed   = false;
          enTriggered = false;
        }
      }
    }
    enLastReading = enReading;

    if (enPressed && !enTriggered && ((nowMs - enStart) >= ENROLL_HOLD_MS)) {
      enTriggered = true;
      if (!g_systemUnlocked) {
        triggerLedAndBuzzer(2, 1200, BUZZ_ALERT, nowMs);
        triggerPopup("ENROLL DITOLAK", "BUKA KUNCI DULU", TFT_RED, 1800);
      } else {
        uint16_t newId = 0;
        char stBuf[32] = "ENROLL...";
        bool ok = enrollFingerprintNextSlot(newId, stBuf, sizeof(stBuf));
        portENTER_CRITICAL(&g_stateMux);
        strncpy(g_fingerStatus, stBuf, sizeof(g_fingerStatus) - 1);
        g_fingerStatus[sizeof(g_fingerStatus) - 1] = '\0';
        g_needFullClear = true;
        portEXIT_CRITICAL(&g_stateMux);
        if (!ok) {
          triggerPopup("ENROLL GAGAL", stBuf, TFT_RED, 1800);
        }
      }
      resetMainButtonState();
    }

    // --- 1. Tombol Emergency Bypass (GPIO 2, tahan 3.5 detik) ---
    bool emReading = digitalRead(PIN_BTN_EMRG);
    if (emReading != emLastReading) emDebMs = nowMs;
    if ((nowMs - emDebMs) > DEBOUNCE_MS) {
      if (emReading != emStableState) {
        emStableState = emReading;
        if (emStableState == LOW) {
          emPressed   = true;
          emTriggered = false;
          emStart     = nowMs;
        } else {
          emPressed   = false;
          emTriggered = false;
        }
      }
    }
    emLastReading = emReading;

    if (emPressed && !emTriggered && ((nowMs - emStart) >= EMERGENCY_HOLD_MS)) {
      emTriggered = true;
      bool newEmg = !g_emergencyByPass;

      portENTER_CRITICAL(&g_stateMux);
      g_emergencyByPass = newEmg;
      if (newEmg) {
        g_bootState   = BOOT_UNLOCKED_ACTIVE;
        g_currentPage = PAGE_HOME;
        memcpy(g_fingerStatus, "EMERGENCY ON", sizeof(g_fingerStatus));
      } else if (!g_systemUnlocked) {
        g_bootState   = BOOT_WAIT_FINGERPRINT;
        g_currentPage = PAGE_HOME;
        g_seatRelayActive = false;
        memcpy(g_fingerStatus, "LOCKED (R503)", sizeof(g_fingerStatus));
      } else {
        memcpy(g_fingerStatus, "UNLOCKED (R503)", sizeof(g_fingerStatus));
      }
      g_needFullClear = true;
      portEXIT_CRITICAL(&g_stateMux);

      applyRelayOutput(nowMs);
      resetMainButtonState();

      if (newEmg) {
        triggerLedAndBuzzer(2, 2000, BUZZ_ALERT, nowMs);
        triggerPopup("EMERGENCY ON", "RELAY BYPASS", TFT_ORANGE, 1800);
      } else if (!g_systemUnlocked) {
        triggerLedAndBuzzer(0, 0, BUZZ_ENROLL_STEP, nowMs);
        setTciGateForPage(false);
        Core0Command cmd = {CMD_SYSTEM_LOCKED, PAGE_HOME, PAGE_HOME, false};
        if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
      }

      if (g_core0CmdQueue) {
        Core0Command emgCmd = {CMD_SAVE_EMG_STATE, PAGE_HOME, PAGE_HOME, newEmg};
        xQueueSend(g_core0CmdQueue, &emgCmd, 0);
      }
    }

    // --- 2. Autentikasi Sidik Jari R503 ---
    unsigned long cooldownUntil = g_fingerCooldownUntilMs.load(std::memory_order_acquire);
    if (!g_systemUnlocked && (nowMs >= cooldownUntil) && (nowMs - lastFingerMs >= 140)) {
      lastFingerMs = nowMs;
      uint16_t fid = 0;
      int conf = 0;
      int scanRes = scanFingerprintOnce(fid, conf);
      if (scanRes == 0) {
        g_fingerFailStreak.store(0, std::memory_order_release);
        char stBuf[32];
        snprintf(stBuf, sizeof(stBuf), "ID#%u (%d)", fid, conf);

        portENTER_CRITICAL(&g_stateMux);
        memcpy(g_fingerStatus, stBuf, sizeof(g_fingerStatus));
        g_systemUnlocked = true;
        g_bootState      = BOOT_UNLOCKED_ACTIVE;
        g_currentPage    = PAGE_HOME;
        g_needFullClear  = true;
        portEXIT_CRITICAL(&g_stateMux);

        applyRelayOutput(nowMs);
        triggerLedAndBuzzer(1, 2000, BUZZ_UNLOCK, nowMs);
        setTciGateForPage(false);
        resetMainButtonState();

        Core0Command cmd = {CMD_PAGE_CHANGED, PAGE_HOME, PAGE_HOME, false};
        if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
      } else if (scanRes == -2 || scanRes == -3) {
        uint8_t fails = static_cast<uint8_t>(g_fingerFailStreak.fetch_add(1, std::memory_order_acq_rel) + 1);
        if (fails >= FINGER_MAX_FAIL_STREAK) {
          g_fingerFailStreak.store(0, std::memory_order_release);
          g_fingerCooldownUntilMs.store(nowMs + FINGER_FAIL_COOLDOWN_MS, std::memory_order_release);
          triggerLedAndBuzzer(2, 2000, BUZZ_ALERT, nowMs);
          triggerPopup("JARI DITOLAK", "TUNGGU 5 DETIK", TFT_RED, 2200);
        }
      }
    }

    // --- 3. Tombol Utama (GPIO 1) ---
    bool canOperateUi = (g_systemUnlocked || g_emergencyByPass);
    if (canOperateUi) {
      bool reading = digitalRead(PIN_BTN_MAIN);
      if (reading != lastReading) lastDebMs = nowMs;

      if ((nowMs - lastDebMs) > DEBOUNCE_MS) {
        if (reading != stableState) {
          stableState = reading;
          if (stableState == LOW) {
            isPressed     = true;
            longPressSent = false;
            pressStartMs  = nowMs;
          } else {
            if (isPressed && !longPressSent) {
              pendingClicks++;
              lastClickRelMs = nowMs;
            }
            if (g_seatRelayActive) {
              setSeatRelayHold(false, nowMs);
            }
            isPressed     = false;
            longPressSent = false;
          }
        }
      }
      lastReading = reading;

      if (isPressed && !longPressSent && ((nowMs - pressStartMs) >= LONG_PRESS_MS)) {
        longPressSent = true;
        pendingClicks = 0;

        if (g_currentPage == PAGE_TCI) {
          Core0Command cmd = {CMD_TCI_NEXT_MODE, PAGE_TCI, PAGE_TCI, false};
          if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
        } else {
          setSeatRelayHold(true, nowMs);
        }
      }

      if (pendingClicks > 0 && !isPressed && ((nowMs - lastClickRelMs) >= DOUBLE_TAP_MS)) {
        uint8_t clicks = pendingClicks;
        pendingClicks = 0;

        PageId prevPage = g_currentPage;
        if (clicks >= 2) {
          if (g_currentPage == PAGE_TCI) {
            setSystemStateAtomic(BOOT_UNLOCKED_ACTIVE, PAGE_HOME, g_systemUnlocked, true);
            setTciGateForPage(false);
            Core0Command cmd = {CMD_PAGE_CHANGED, prevPage, PAGE_HOME, false};
            if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
          } else {
            PageId nextP = (g_currentPage == PAGE_HOME) ? PAGE_NAV : PAGE_HOME;
            setSystemStateAtomic(BOOT_UNLOCKED_ACTIVE, nextP, g_systemUnlocked, true);
            setTciGateForPage(false);
            Core0Command cmd = {CMD_PAGE_CHANGED, prevPage, nextP, false};
            if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
          }
        } else {
          if (g_currentPage == PAGE_HOME) {
            setSystemStateAtomic(BOOT_UNLOCKED_ACTIVE, PAGE_NAV, g_systemUnlocked, true);
            setTciGateForPage(false);
            Core0Command cmd = {CMD_PAGE_CHANGED, prevPage, PAGE_NAV, false};
            if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
          } else if (g_currentPage == PAGE_NAV) {
            setSystemStateAtomic(BOOT_UNLOCKED_ACTIVE, PAGE_TCI, g_systemUnlocked, true);
            setTciGateForPage(true);
            Core0Command cmd = {CMD_PAGE_CHANGED, prevPage, PAGE_TCI, false};
            if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
          } else if (g_currentPage == PAGE_TCI) {
            Core0Command cmd = {CMD_SUBPAGE_TOGGLE, PAGE_TCI, PAGE_TCI, false};
            if (g_core0CmdQueue) xQueueSend(g_core0CmdQueue, &cmd, 0);
          }
        }
      }
    }

    applyRelayOutput(nowMs);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void taskDisplayLcdCore0(void* pvParameters) {
  (void)pvParameters;
  esp_task_wdt_add(nullptr);

  unsigned long lastRtcReadMs  = 0;
  unsigned long lastNavPollMs  = 0;
  unsigned long lastNotifPollMs = 0;

  BootState lastRenderedBoot   = static_cast<BootState>(255);
  PageId    lastRenderedPage   = static_cast<PageId>(255);
  uint8_t   lastRenderedSub    = 255;
  bool      lastRenderedRelay  = false;
  bool      lastRenderedEmg    = false;
  bool      lastRenderedOnline = false;
  bool      lastRenderedR503   = false;
  uint8_t   lastRenderedR503Err = 255;
  bool      lastRenderedRtc    = false;
  bool      lastRenderedRtcLp  = false;
  bool      lastPopupActive    = false;
  bool      lastNotifActive    = false;
  uint32_t  lastPopupSeq       = 0;
  uint32_t  lastNotifSeq       = 0;
  uint32_t  lastRtcSeq         = 0;
  uint32_t  lastTciSeq         = 0;
  uint32_t  lastNavSeq         = 0;

  for (;;) {
    esp_task_wdt_reset();
    unsigned long nowMs = millis();

    watch.loop();
    processIncomingEspNowFrames();
    syncPendingEspNowPeer();

    if (nowMs - lastNavPollMs >= NAV_POLL_MS) {
      lastNavPollMs = nowMs;
      updateNavFromChronos();
    }

    if (nowMs - lastNotifPollMs >= NOTIF_POLL_MS) {
      lastNotifPollMs = nowMs;
      pollChronosNotification();
    }

    Core0Command cmd;
    while (g_core0CmdQueue && xQueueReceive(g_core0CmdQueue, &cmd, 0) == pdTRUE) {
      if (cmd.type == CMD_SAVE_EMG_STATE) {
        saveEmergencyStateToNvs(cmd.flagVal);
      } else if (cmd.type == CMD_PAGE_CHANGED) {
        if (cmd.nextPage == PAGE_TCI) {
          initTciEspNowOnce();
          setTciSubPageAtomic(0, false);
        }
        startComponentTransition(8);
        lastRenderedPage = static_cast<PageId>(255);
      } else if (cmd.type == CMD_SUBPAGE_TOGGLE) {
        uint8_t nextSub = static_cast<uint8_t>((getTciSubPageAtomic() + 1) % 2);
        setTciSubPageAtomic(nextSub, false);
        startComponentTransition(8);
        lastRenderedSub = 255;
      } else if (cmd.type == CMD_TCI_NEXT_MODE) {
        TelemetryData snap;
        unsigned long lastRx = 0;
        portENTER_CRITICAL(&g_tciMux);
        snap   = g_tciPkt;
        lastRx = g_lastTciRxMs;
        portEXIT_CRITICAL(&g_tciMux);

        bool online = (lastRx > 0) && ((nowMs - lastRx) < TCI_TIMEOUT_MS);
        if (!online) {
          triggerPopup("NO SIGNAL", "C3 / TCI OFFLINE", TFT_RED, 1500);
        } else if (snap.rpm > IDLE_MAX_RPM_MODE) {
          char rpmLine[20];
          snprintf(rpmLine, sizeof(rpmLine), "RPM: %u (>400)", snap.rpm);
          triggerPopup("DITOLAK!", rpmLine, TFT_RED, 1600);
        } else {
          uint8_t nextM = static_cast<uint8_t>((snap.mode + 1) % 4);
          portENTER_CRITICAL(&g_stateMux);
          g_desiredMode       = nextM;
          g_pendingModeChange = true;
          g_modeRetryCount    = 0;
          g_lastModeSendMs    = 0;
          portEXIT_CRITICAL(&g_stateMux);

          const char* mNames[4] = {"DAILY", "RACING", "EXTREME", "CUSTOM"};
          char l2[20];
          snprintf(l2, sizeof(l2), "%s (%u RPM)", mNames[nextM], getRedlineRPM(nextM));
          triggerPopup("GANTI MODE...", l2, TFT_YELLOW, 1500);
        }
      } else if (cmd.type == CMD_SYSTEM_LOCKED) {
        setTciSubPageAtomic(0, false);
        startComponentTransition(8);
        lastRenderedBoot = static_cast<BootState>(255);
      }
    }

    bool pendingModeSnap = false;
    uint8_t desiredModeSnap = 0;
    unsigned long lastSendSnap = 0;
    uint8_t retrySnap = 0;
    bool sendCbSeenSnap = false;
    bool lastSendOkSnap = false;

    portENTER_CRITICAL(&g_stateMux);
    pendingModeSnap = g_pendingModeChange;
    desiredModeSnap = g_desiredMode;
    lastSendSnap    = g_lastModeSendMs;
    retrySnap       = g_modeRetryCount;
    sendCbSeenSnap  = g_espSendCbSeen;
    lastSendOkSnap  = g_lastEspSendOk;
    portEXIT_CRITICAL(&g_stateMux);

    if (isUnlockedAndOnTciPage() && pendingModeSnap) {
      TelemetryData snap;
      unsigned long lastRx = 0;
      portENTER_CRITICAL(&g_tciMux);
      snap   = g_tciPkt;
      lastRx = g_lastTciRxMs;
      portEXIT_CRITICAL(&g_tciMux);

      bool online = (lastRx > 0) && ((nowMs - lastRx) < TCI_TIMEOUT_MS);
      if (!online || snap.rpm > IDLE_MAX_RPM_MODE) {
        portENTER_CRITICAL(&g_stateMux);
        g_pendingModeChange = false;
        portEXIT_CRITICAL(&g_stateMux);
      } else if (snap.mode == desiredModeSnap && (snap.ackStatus == 1 || snap.ackStatus == 3)) {
        portENTER_CRITICAL(&g_stateMux);
        g_pendingModeChange = false;
        portEXIT_CRITICAL(&g_stateMux);
        const char* mNames[4] = {"DAILY", "RACING", "EXTREME", "CUSTOM"};
        char l2[20];
        snprintf(l2, sizeof(l2), "%s (%u RPM)", mNames[desiredModeSnap], getRedlineRPM(desiredModeSnap));
        triggerPopup("MODE AKTIF:", l2, TFT_GREEN, 1600);
      } else if (sendCbSeenSnap && !lastSendOkSnap) {
        portENTER_CRITICAL(&g_stateMux);
        g_espSendCbSeen = false;
        if (retrySnap < MODE_SEND_MAX_RETRY) {
          g_modeRetryCount = static_cast<uint8_t>(retrySnap + 1);
          portEXIT_CRITICAL(&g_stateMux);
          sendModePacket(desiredModeSnap);
        } else {
          g_pendingModeChange = false;
          portEXIT_CRITICAL(&g_stateMux);
          triggerPopup("GAGAL KIRIM", "C3 TIDAK MERESPON", TFT_RED, 1600);
        }
      } else if ((nowMs - lastSendSnap) >= 450) {
        portENTER_CRITICAL(&g_stateMux);
        if (retrySnap < MODE_SEND_MAX_RETRY) {
          g_modeRetryCount = static_cast<uint8_t>(retrySnap + 1);
          portEXIT_CRITICAL(&g_stateMux);
          sendModePacket(desiredModeSnap);
        } else {
          g_pendingModeChange = false;
          portEXIT_CRITICAL(&g_stateMux);
          triggerPopup("GAGAL MODE", "TIMEOUT ACK ECU", TFT_RED, 1600);
        }
      }
    }

    if (nowMs - lastRtcReadMs >= 500) {
      lastRtcReadMs = nowMs;
      updateRtcCache();
      updateAdaptiveBacklight();
    }

    BootState curBoot;
    PageId    curPage;
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

    portENTER_CRITICAL(&g_stateMux);
    curBoot     = g_bootState;
    curPage     = g_currentPage;
    curUnlocked = g_systemUnlocked;
    curEmg      = g_emergencyByPass;
    fullClear   = g_needFullClear;
    if (!g_uiTrans.active) {
      g_needFullClear = false;
    }
    memcpy(tBuf, g_timeStr, sizeof(tBuf));
    memcpy(dBuf, g_dateStr, sizeof(dBuf));
    memcpy(fBuf, g_fingerStatus, sizeof(fBuf));
    chipTempC   = g_chipTempC;
    rtcSeq      = g_rtcSeq;

    popStart    = g_popupStartMs;
    popDur      = g_popupDurationMs;
    popSeq      = g_popupSeq;
    memcpy(popL1, g_popupLine1, sizeof(popL1));
    memcpy(popL2, g_popupLine2, sizeof(popL2));
    popCol      = g_popupBgColor;
    portEXIT_CRITICAL(&g_stateMux);

    bool relayOn     = (curUnlocked || curEmg);
    bool popupActive = isTimerActive(popStart, popDur, nowMs);

    NotifPopupData notifSnap;
    portENTER_CRITICAL(&g_notifMux);
    if (g_notifPopup.active && !isTimerActive(g_notifPopup.startMs, g_notifPopup.durationMs, nowMs)) {
      g_notifPopup.active = false;
    }
    notifSnap = g_notifPopup;
    portEXIT_CRITICAL(&g_notifMux);

    bool notifActive = notifSnap.active && (curBoot == BOOT_UNLOCKED_ACTIVE) && (curPage != PAGE_TCI);

    if (popupActive) {
      if (!lastPopupActive || popSeq != lastPopupSeq) {
        drawTciPopup(popL1, popL2, popCol);
        lastPopupSeq = popSeq;
      }
      lastPopupActive = true;
      vTaskDelay(pdMS_TO_TICKS(16));
      continue;
    } else if (lastPopupActive) {
      lastPopupActive = false;
      fullClear       = true;
    }

    if (notifActive) {
      if (!lastNotifActive || notifSnap.seq != lastNotifSeq) {
        drawNotifPopup(notifSnap);
        lastNotifSeq = notifSnap.seq;
      }
      lastNotifActive = true;
      vTaskDelay(pdMS_TO_TICKS(16));
      continue;
    } else if (lastNotifActive) {
      lastNotifActive = false;
      fullClear       = true;
    }

    if (g_uiTrans.active) {
      if ((nowMs - g_uiTrans.lastStepMs) < g_uiTrans.stepDelayMs) {
        vTaskDelay(pdMS_TO_TICKS(8));
        continue;
      }
      g_uiTrans.lastStepMs = nowMs;

      if (!g_uiTrans.exitDone) {
        animateExitComponents(g_uiTrans.step);
        g_uiTrans.step++;
        if (g_uiTrans.step >= 2) {
          g_uiTrans.exitDone = true;
          g_uiTrans.step     = 0;
          portENTER_CRITICAL(&g_stateMux);
          g_needFullClear = false;
          portEXIT_CRITICAL(&g_stateMux);
        }
        vTaskDelay(pdMS_TO_TICKS(8));
        continue;
      } else {
        uint8_t stage = g_uiTrans.step;
        animateEnterComponentStage(stage);

        if (curBoot == BOOT_WAIT_FINGERPRINT) {
          bool r503Ok = g_r503Ok.load(std::memory_order_acquire);
          uint8_t r503Err = g_lastR503Err.load(std::memory_order_acquire);
          bool rtcOk = g_rtcOk.load(std::memory_order_acquire);
          bool rtcLp = g_rtcLostPower.load(std::memory_order_acquire);
          renderBootLockScreen(false, curEmg, r503Ok, r503Err, rtcOk, rtcLp, stage);
        } else if (curPage == PAGE_HOME) {
          bool rtcOk = g_rtcOk.load(std::memory_order_acquire);
          bool rtcLp = g_rtcLostPower.load(std::memory_order_acquire);
          renderPageHome(false, relayOn, curEmg, rtcOk, rtcLp, tBuf, dBuf, fBuf, chipTempC, stage);
        } else if (curPage == PAGE_NAV) {
          NavTelemetryData navSnap;
          portENTER_CRITICAL(&g_navMux);
          navSnap = g_navData;
          portEXIT_CRITICAL(&g_navMux);
          bool navOnline = navSnap.bleConnected && navSnap.navActive &&
                           (navSnap.lastRxMs > 0) && ((nowMs - navSnap.lastRxMs) < NAV_TIMEOUT_MS);
          renderPageNav(false, relayOn, curEmg, navSnap, navOnline, stage);
        } else if (curPage == PAGE_TCI) {
          TelemetryData snap;
          unsigned long lastRx;
          uint8_t histCopy[ADV_HIST_LEN];
          int hIdx;
          portENTER_CRITICAL(&g_tciMux);
          snap   = g_tciPkt;
          lastRx = g_lastTciRxMs;
          memcpy(histCopy, g_advHistory, ADV_HIST_LEN);
          hIdx   = g_advHistIdx;
          portEXIT_CRITICAL(&g_tciMux);

          bool online = (lastRx > 0) && ((nowMs - lastRx) < TCI_TIMEOUT_MS);
          uint8_t subPage = getTciSubPageAtomic();
          if (subPage == 0) {
            renderPageTciDashboard(false, relayOn, curEmg, snap, online, stage);
          } else {
            renderPageTciCurve(false, relayOn, curEmg, snap, histCopy, hIdx, online, stage);
          }
        }

        g_uiTrans.step++;
        if (g_uiTrans.step >= 3) {
          g_uiTrans.active    = false;
          g_uiTrans.mode      = UI_TRANS_NONE;
          lastRenderedBoot    = curBoot;
          lastRenderedPage    = curPage;
          lastRenderedSub     = getTciSubPageAtomic();
          lastRenderedRelay   = relayOn;
          lastRenderedEmg     = curEmg;
        }
        vTaskDelay(pdMS_TO_TICKS(8));
        continue;
      }
    }

    if (curBoot == BOOT_WAIT_FINGERPRINT) {
      bool r503Ok = g_r503Ok.load(std::memory_order_acquire);
      uint8_t r503Err = g_lastR503Err.load(std::memory_order_acquire);
      bool rtcOk = g_rtcOk.load(std::memory_order_acquire);
      bool rtcLp = g_rtcLostPower.load(std::memory_order_acquire);
      bool changed = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedEmg != curEmg) ||
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
      bool rtcOk = g_rtcOk.load(std::memory_order_acquire);
      bool rtcLp = g_rtcLostPower.load(std::memory_order_acquire);
      bool pageChanged = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedPage != curPage) ||
                         (lastRenderedRelay != relayOn) || (lastRenderedEmg != curEmg) ||
                         (lastRenderedRtc != rtcOk) || (lastRenderedRtcLp != rtcLp);
      if (pageChanged || rtcSeq != lastRtcSeq) {
        renderPageHome(pageChanged, relayOn, curEmg, rtcOk, rtcLp, tBuf, dBuf, fBuf, chipTempC, 255);
        lastRenderedBoot  = curBoot;
        lastRenderedPage  = curPage;
        lastRenderedRelay = relayOn;
        lastRenderedEmg   = curEmg;
        lastRenderedRtc   = rtcOk;
        lastRenderedRtcLp = rtcLp;
        lastRtcSeq        = rtcSeq;
      }
    } else if (curPage == PAGE_NAV) {
      NavTelemetryData navSnap;
      portENTER_CRITICAL(&g_navMux);
      navSnap = g_navData;
      portEXIT_CRITICAL(&g_navMux);

      bool navOnline = navSnap.bleConnected && navSnap.navActive &&
                       (navSnap.lastRxMs > 0) && ((nowMs - navSnap.lastRxMs) < NAV_TIMEOUT_MS);
      bool pageChanged = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedPage != curPage) ||
                         (lastRenderedRelay != relayOn) || (lastRenderedEmg != curEmg);
      if (pageChanged || navSnap.seq != lastNavSeq || navOnline != lastRenderedOnline) {
        renderPageNav(pageChanged, relayOn, curEmg, navSnap, navOnline, 255);
        lastRenderedBoot   = curBoot;
        lastRenderedPage   = curPage;
        lastRenderedRelay  = relayOn;
        lastRenderedEmg    = curEmg;
        lastRenderedOnline = navOnline;
        lastNavSeq         = navSnap.seq;
      }
    } else if (curPage == PAGE_TCI) {
      TelemetryData snap;
      unsigned long lastRx;
      uint32_t      tciSeq;
      uint8_t       histCopy[ADV_HIST_LEN];
      int           hIdx;

      portENTER_CRITICAL(&g_tciMux);
      snap   = g_tciPkt;
      lastRx = g_lastTciRxMs;
      tciSeq = g_tciRxSeq;
      memcpy(histCopy, g_advHistory, ADV_HIST_LEN);
      hIdx   = g_advHistIdx;
      portEXIT_CRITICAL(&g_tciMux);

      bool online = (lastRx > 0) && ((nowMs - lastRx) < TCI_TIMEOUT_MS);
      uint8_t subPage = getTciSubPageAtomic();
      bool pageChanged = fullClear || (lastRenderedBoot != curBoot) || (lastRenderedPage != curPage) ||
                         (lastRenderedSub != subPage) || (lastRenderedRelay != relayOn) ||
                         (lastRenderedEmg != curEmg);
      if (pageChanged || online != lastRenderedOnline || tciSeq != lastTciSeq) {
        if (subPage == 0) {
          renderPageTciDashboard(pageChanged, relayOn, curEmg, snap, online, 255);
        } else {
          renderPageTciCurve(pageChanged, relayOn, curEmg, snap, histCopy, hIdx, online, 255);
        }
        lastRenderedBoot   = curBoot;
        lastRenderedPage   = curPage;
        lastRenderedSub    = subPage;
        lastRenderedRelay  = relayOn;
        lastRenderedEmg    = curEmg;
        lastRenderedOnline = online;
        lastTciSeq         = tciSeq;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(16));
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  pinMode(PIN_RELAY_SEAT, OUTPUT);
  digitalWrite(PIN_RELAY_SEAT, LOW);

  pinMode(PIN_LED_NOTIF, OUTPUT);
  digitalWrite(PIN_LED_NOTIF, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_BTN_MAIN, INPUT_PULLUP);
  pinMode(PIN_BTN_EMRG, INPUT_PULLUP);
  pinMode(PIN_BTN_ENROLL, INPUT_PULLUP);

  Serial.begin(115200);

  g_i2cMutex  = xSemaphoreCreateMutex();
  g_nvsMutex  = xSemaphoreCreateMutex();
  g_r503Mutex = xSemaphoreCreateMutex();
  g_core0CmdQueue = xQueueCreate(16, sizeof(Core0Command));
  g_espNowRxQueue = xQueueCreate(16, sizeof(EspNowRawFrame));

  if (!g_i2cMutex || !g_nvsMutex || !g_r503Mutex || !g_core0CmdQueue || !g_espNowRxQueue) {
    Serial.println("[FATAL] Alokasi RTOS gagal!");
    delay(1000);
    ESP.restart();
  }

  loadPersistentSettings();

  if (g_emergencyByPass) {
    setSystemStateAtomic(BOOT_UNLOCKED_ACTIVE, PAGE_HOME, false, true);
  } else {
    setSystemStateAtomic(BOOT_WAIT_FINGERPRINT, PAGE_HOME, false, true);
  }
  applyRelayOutput();
  setTciGateForPage(false);

  initLcdBacklightPwm();
  lcd.init();
  lcd.setRotation(1);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(45, 110);
  lcd.print("BOOTING SYSTEM...");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(50);
  if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    if (rtc.begin(&Wire)) {
      g_rtcOk.store(true, std::memory_order_release);
      if (rtc.lostPower()) {
        g_rtcLostPower.store(true, std::memory_order_release);
      }
    }
    xSemaphoreGive(g_i2cMutex);
  }
  updateRtcCache();
  updateAdaptiveBacklight();

  r503Serial.begin(57600, SERIAL_8N1, PIN_R503_RX, PIN_R503_TX);
  if (g_r503Mutex && xSemaphoreTake(g_r503Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    bool r503Ready = false;
    for (int attempt = 0; attempt < 3; attempt++) {
      if (finger.verifyPassword()) {
        r503Ready = true;
        break;
      }
      delay(40);
    }
    g_r503Ok.store(r503Ready, std::memory_order_release);
    if (!r503Ready) {
      g_lastR503Err.store(FINGERPRINT_PACKETRECIEVEERR, std::memory_order_release);
    }
    xSemaphoreGive(g_r503Mutex);
  }

  watch.begin();
  watch.set24Hour(true);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif

  xTaskCreatePinnedToCore(
    taskImmobilizerCore1,
    "ImmobCore1",
    4096,
    nullptr,
    4,
    nullptr,
    CORE_SECURITY
  );

  xTaskCreatePinnedToCore(
    taskDisplayLcdCore0,
    "LcdCommCore0",
    12288,
    nullptr,
    2,
    nullptr,
    CORE_COMM_UI
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
