/* ==============================================================================
 * FIRMWARE ECU TCI SATRIA FU KARBURATOR (ESP32-S3) - DEDICATED ECU ENGINE NODE
 * LEVEL: OEM / AUTOMOTIVE GRADE (DAILY-USE HARDENED v12)
 * ============================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "soc/gpio_reg.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "common_proto.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define ADC_ATTEN_TARGET ADC_ATTEN_DB_12
#else
#define ADC_ATTEN_TARGET ADC_ATTEN_DB_11
#endif

static const char *TAG = "ECU_TCI";

// --- KONFIGURASI PIN HARDWARE ESP32-S3 ---
#define PIN_PULSER       GPIO_NUM_14
#define PIN_TCI          GPIO_NUM_18
#define PIN_RX_TELEMETRY GPIO_NUM_19
#define PIN_TX_TELEMETRY GPIO_NUM_20
#define PIN_TACH_OUT     GPIO_NUM_42

#define TCI_COIL_CHARGE() REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << PIN_TCI))
#define TCI_COIL_SPARK()  REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << PIN_TCI))

#define ADC_CHAN_TPS     ADC_CHANNEL_0  // GPIO 1 (ADC1)
#define ADC_CHAN_TEMP    ADC_CHANNEL_1  // GPIO 2 (ADC1)
#define ADC_CHAN_BATT    ADC_CHANNEL_3  // GPIO 4 (ADC1) - pengganti GPIO 3

#define WDT_TIMEOUT_MS         600
#define NVS_NAMESPACE          "ecu_tci_cfg"
#define ENGINE_STALL_TIMEOUT_US 300000UL  // 300 ms tanpa pulsa (<200 RPM) -> mesin mati
#define DEFAULT_DAILY_RPM_LIMIT 11500U    // Batas harian aman untuk mesin standar/harian

// --- DEFINISI KODE KERUSAKAN HARDWARE (BITMASK) ---
#define FAULT_NONE         0
#define FAULT_TPS_OUT      (1 << 0)
#define FAULT_NTC_OUT      (1 << 1)
#define FAULT_BATT_LOW     (1 << 2)
#define FAULT_BATT_HIGH    (1 << 3)
#define FAULT_NVS_ERR      (1 << 4)
#define FAULT_ADC_ERR      (1 << 5)

#define SENSOR_FAULT_MASK  (FAULT_TPS_OUT | FAULT_NTC_OUT | FAULT_BATT_LOW | FAULT_BATT_HIGH | FAULT_ADC_ERR)

#define ABS_VAL(x)         ((x) < 0 ? -(x) : (x))
static inline int64_t clamp_i64(int64_t x, int64_t min_v, int64_t max_v) {
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}
#define CLAMP(x, min, max) ((__typeof__(x))clamp_i64((int64_t)(x), (int64_t)(min), (int64_t)(max)))
#define INV_3600_Q32       1193046ULL
#define MAX_DWELL_US       6000U

typedef enum { STATE_STOPPED = 0, STATE_CRANKING, STATE_RUNNING, STATE_LIMP_HOME } EngineState;
typedef enum { MODE_DAILY = 0, MODE_RACING, MODE_EXTREME, MODE_CUSTOM } ModePengapian;
typedef enum { BBM_PERTALITE = 0, BBM_PERTAMAX } JenisBBM;
typedef enum { IGN_IDLE = 0, IGN_WAITING_CHARGE, IGN_WAITING_SPARK } IgnitionSequenceState;

// --- RAM MEMORY SCRUBBING & E2E DATA PROTECTION ---
portMUX_TYPE secure_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    uint32_t data;
    uint32_t data_inverted;
} SecureData_t;

SecureData_t sec_mode;
SecureData_t sec_engine_state;

void IRAM_ATTR SecureData_Write(SecureData_t* sec, uint32_t value) {
    portENTER_CRITICAL_SAFE(&secure_mux);
    sec->data = value;
    sec->data_inverted = ~value;
    portEXIT_CRITICAL_SAFE(&secure_mux);
}

bool IRAM_ATTR SecureData_Read(SecureData_t* sec, uint32_t* out_value) {
    uint32_t d, d_inv;
    portENTER_CRITICAL_SAFE(&secure_mux);
    d = sec->data;
    d_inv = sec->data_inverted;
    portEXIT_CRITICAL_SAFE(&secure_mux);

    if (d == ~d_inv) {
        *out_value = d;
        return true;
    }
    return false;
}

// --- VARIABEL GLOBAL ATOMIK (LOCK-FREE RTOS & ISR SAFE) ---
_Atomic uint16_t currentRPM = 0;
_Atomic uint8_t currentTPS = 0;
_Atomic uint16_t currentBatteryVoltage10 = 126;
_Atomic int16_t currentEngineTemp10 = 300;
_Atomic int16_t currentDegree10 = 100;
_Atomic bool sensorFaultStatus = false;
_Atomic uint16_t hardwareFaultCode = FAULT_NONE;
_Atomic uint8_t activeMapIndex = 0;
_Atomic uint8_t currentCustomSlot = 0;
_Atomic uint8_t lastAckStatus = 0; // 0: none, 1: ok, 2: crc/err, 3: RAM applied (pending NVS), 4: preload sync ok
_Atomic bool pendingModeSaveToNvs = false;
_Atomic uint8_t pendingModeValue = 0;
_Atomic bool pendingSlotSaveToNvs = false;
_Atomic uint8_t pendingSlotValue = 0;

_Atomic uint32_t pulse_interval_us = 0;
_Atomic uint32_t last_pulse_time_us = 0;
_Atomic int16_t cachedAdv10 = 100;
_Atomic uint32_t cachedDwellUs = 3200;
_Atomic uint16_t activeCustomRpmLimit = DEFAULT_DAILY_RPM_LIMIT;

_Atomic uint8_t ignState = IGN_IDLE;
_Atomic uint64_t target_spark_count = 0;
_Atomic uint32_t charge_start_time_us = 0;
_Atomic uint8_t currentFuel = BBM_PERTALITE;
_Atomic uint8_t limp_cut_counter = 0;
_Atomic uint8_t rev_cut_counter = 0;

TaskHandle_t ignCalcTaskHandle = NULL;
gptimer_handle_t timerIgnition = NULL;
adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
#define MAX_CUSTOM_SLOTS 5

// --- STRUKTUR PAKET KOMUNIKASI UART (100% SINKRON DENGAN ESP32-C3) ---
#pragma pack(push, 1)
typedef struct {
  uint16_t header;     // 0xAA55
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
} TelemetryData;       // Total: 19 Bytes

typedef struct {
  uint16_t header;                                 // 0x55CC
  uint8_t  cmdType;                                // 0x01: Mode, 0x02: RAM Map, 0x03: Save NVS, 0x04: Preload Sync
  uint8_t  slotOrMode;
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; // 150 Bytes
  uint16_t dwellData[NUM_RPM_POINTS];               // 30 Bytes
  uint16_t rpmLimit;                                // 2 Bytes
  uint16_t crc16;                                   // 2 Bytes
} CommandPacket;                                    // Total: 188 Bytes
#pragma pack(pop)

_Static_assert(sizeof(TelemetryData) == 19, "Ukuran TelemetryData harus 19 byte");
_Static_assert(sizeof(CommandPacket) == 188, "Ukuran CommandPacket harus 188 byte");

const int16_t ROTOR_PULSER_DEGREES_10 = 350;

DRAM_ATTR const uint16_t rpmAxis[NUM_RPM_POINTS] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000 };
DRAM_ATTR const uint8_t  tpsAxis[NUM_TPS_POINTS] = { 0, 25, 50, 75, 100 };

DRAM_ATTR const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};

DRAM_ATTR const int16_t mapRacing3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {110, 110, 110, 110, 110}, {120, 120, 125, 130, 130}, {160, 175, 190, 200, 210},
  {210, 230, 250, 260, 270}, {250, 275, 290, 310, 320}, {280, 305, 330, 340, 350},
  {310, 335, 360, 370, 370}, {330, 355, 380, 390, 390}, {340, 365, 390, 390, 390},
  {340, 365, 390, 390, 390}, {330, 355, 380, 380, 380}, {310, 335, 360, 360, 360},
  {290, 315, 340, 340, 340}, {270, 280, 310, 310, 310}, {260, 260, 280, 280, 280}
};

DRAM_ATTR const int16_t mapExtreme3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {120, 120, 120, 120, 120}, {130, 135, 140, 145, 150}, {180, 195, 210, 225, 240},
  {230, 255, 280, 295, 310}, {280, 310, 330, 350, 360}, {310, 340, 370, 385, 400},
  {340, 375, 400, 410, 410}, {360, 395, 420, 430, 430}, {370, 405, 430, 430, 430},
  {370, 405, 430, 430, 430}, {360, 395, 420, 420, 420}, {340, 375, 400, 400, 400},
  {320, 350, 380, 380, 380}, {300, 310, 340, 340, 340}, {280, 280, 300, 300, 300}
};

DRAM_ATTR const uint16_t defaultDwellPerRPM[NUM_RPM_POINTS] = {
  3500, 3500, 3400, 3300, 3200, 3100, 3000, 2900, 2800, 2700, 2600, 2500, 2400, 2300, 2200
};

DRAM_ATTR int16_t mapCustomSlots[MAX_CUSTOM_SLOTS][NUM_RPM_POINTS][NUM_TPS_POINTS];
DRAM_ATTR uint16_t dwellCustomSlots[MAX_CUSTOM_SLOTS][NUM_RPM_POINTS];
DRAM_ATTR uint16_t rpmLimitCustomSlots[MAX_CUSTOM_SLOTS];

DRAM_ATTR int16_t activeCustomMapBuffer[2][NUM_RPM_POINTS][NUM_TPS_POINTS];
DRAM_ATTR uint16_t activeCustomDwellBuffer[2][NUM_RPM_POINTS];

static inline void setFaultBit(uint16_t bit) {
    atomic_fetch_or(&hardwareFaultCode, bit);
}

static inline void clearFaultBit(uint16_t bit) {
    atomic_fetch_and(&hardwareFaultCode, (uint16_t)(~bit));
}

static int32_t adc_raw_to_mv_fallback(int raw) {
    if (raw <= 0) return 0;
    if (raw >= 4095) return 3300;
    int32_t mv = (raw * 3300) / 4095;
    if (raw < 200) {
        mv = (raw * 160) / 200;
    } else if (raw > 3800) {
        mv = 3060 + ((raw - 3800) * 240) / 295;
    }
    return mv;
}

static inline bool isEngineStopped(void) {
  uint32_t engineState;
  if (SecureData_Read(&sec_engine_state, &engineState)) {
      return (engineState == STATE_STOPPED && atomic_load(&currentRPM) == 0);
  }
  return false;
}

int32_t mapRange(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  if (in_max == in_min) return out_min;
  int32_t result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return CLAMP(result, out_min, out_max);
}

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void updateActiveMapBuffer(void) {
  uint8_t slot = atomic_load(&currentCustomSlot);
  if (slot >= MAX_CUSTOM_SLOTS) slot = 0;
  uint8_t nextBuf = (uint8_t)(1 - atomic_load(&activeMapIndex));

  portENTER_CRITICAL_SAFE(&secure_mux);
  memcpy(activeCustomMapBuffer[nextBuf], mapCustomSlots[slot], sizeof(activeCustomMapBuffer[0]));
  memcpy(activeCustomDwellBuffer[nextBuf], dwellCustomSlots[slot], sizeof(activeCustomDwellBuffer[0]));
  uint16_t limit = rpmLimitCustomSlots[slot];
  portEXIT_CRITICAL_SAFE(&secure_mux);

  atomic_store(&activeCustomRpmLimit, limit);
  atomic_store(&activeMapIndex, nextBuf);
}

esp_err_t nvs_save_custom_slot(uint8_t slot) {
    if (slot >= MAX_CUSTOM_SLOTS) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        setFaultBit(FAULT_NVS_ERR);
        return err;
    }

    char k_map[16], k_lim[16], k_dwl[16];
    snprintf(k_map, sizeof(k_map), "map_s%d", slot);
    snprintf(k_lim, sizeof(k_lim), "lim_s%d", slot);
    snprintf(k_dwl, sizeof(k_dwl), "dwl_s%d", slot);

    err = nvs_set_blob(h, k_map, mapCustomSlots[slot], sizeof(mapCustomSlots[slot]));
    if (err == ESP_OK) err = nvs_set_blob(h, k_dwl, dwellCustomSlots[slot], sizeof(dwellCustomSlots[slot]));
    if (err == ESP_OK) err = nvs_set_u16(h, k_lim, rpmLimitCustomSlots[slot]);
    if (err == ESP_OK) err = nvs_set_u8(h, "act_slot", slot);
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    if (err != ESP_OK) setFaultBit(FAULT_NVS_ERR);
    else clearFaultBit(FAULT_NVS_ERR);
    return err;
}

esp_err_t nvs_save_mode(uint8_t mode) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        setFaultBit(FAULT_NVS_ERR);
        return err;
    }
    err = nvs_set_u8(h, "act_mode", mode);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) setFaultBit(FAULT_NVS_ERR);
    else clearFaultBit(FAULT_NVS_ERR);
    return err;
}

void initNvsAndLoadMaps(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  nvs_handle_t h;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
      for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
          char k_map[16], k_lim[16], k_dwl[16];
          snprintf(k_map, sizeof(k_map), "map_s%d", i);
          snprintf(k_lim, sizeof(k_lim), "lim_s%d", i);
          snprintf(k_dwl, sizeof(k_dwl), "dwl_s%d", i);

          size_t szMap = sizeof(mapCustomSlots[i]);
          if (nvs_get_blob(h, k_map, mapCustomSlots[i], &szMap) != ESP_OK || szMap != sizeof(mapCustomSlots[i])) {
              memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
          }

          size_t szDwl = sizeof(dwellCustomSlots[i]);
          if (nvs_get_blob(h, k_dwl, dwellCustomSlots[i], &szDwl) != ESP_OK || szDwl != sizeof(dwellCustomSlots[i])) {
              memcpy(&dwellCustomSlots[i], &defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
          }

          uint16_t lim = DEFAULT_DAILY_RPM_LIMIT;
          if (nvs_get_u16(h, k_lim, &lim) != ESP_OK || lim < 1000 || lim > 15000) {
              lim = DEFAULT_DAILY_RPM_LIMIT;
          }
          rpmLimitCustomSlots[i] = lim;
      }

      uint8_t savedSlot = 0;
      if (nvs_get_u8(h, "act_slot", &savedSlot) == ESP_OK && savedSlot < MAX_CUSTOM_SLOTS) {
          atomic_store(&currentCustomSlot, savedSlot);
      }

      uint8_t savedMode = MODE_DAILY;
      if (nvs_get_u8(h, "act_mode", &savedMode) == ESP_OK && savedMode <= MODE_CUSTOM) {
          SecureData_Write(&sec_mode, savedMode);
      }
      nvs_close(h);
  } else {
      for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
        memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
        memcpy(&dwellCustomSlots[i], &defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
        rpmLimitCustomSlots[i] = DEFAULT_DAILY_RPM_LIMIT;
      }
  }
  updateActiveMapBuffer();
}

typedef struct {
    int16_t finalAdvance10;
    uint32_t finalDwellUs;
} IgnitionCommand_t;

IgnitionCommand_t HAL_IgnitionLogic_Calculate(uint16_t rpm, uint8_t tps, int16_t temp10, uint16_t batt10, uint8_t fuel, ModePengapian mode, EngineState state, bool fault) {
    IgnitionCommand_t cmd;

    uint16_t calcRpm = CLAMP(rpm, 0, rpmAxis[NUM_RPM_POINTS - 1]);
    uint8_t calcTps = CLAMP(tps, 0, 100);

    uint8_t r0 = (uint8_t)(calcRpm / 1000);
    if (r0 >= NUM_RPM_POINTS - 1) r0 = NUM_RPM_POINTS - 2;
    uint8_t t0 = (uint8_t)(calcTps / 25);
    if (t0 >= NUM_TPS_POINTS - 1) t0 = NUM_TPS_POINTS - 2;
    uint8_t r1 = (uint8_t)(r0 + 1), t1 = (uint8_t)(t0 + 1);
    int32_t rF = ((int32_t)(calcRpm - (r0 * 1000)) * 1024) / 1000;

    if (mode == MODE_CUSTOM) {
        uint8_t bufIdx = atomic_load(&activeMapIndex);
        uint16_t d0 = activeCustomDwellBuffer[bufIdx][r0];
        uint16_t d1 = activeCustomDwellBuffer[bufIdx][r1];
        uint32_t baseDwell = (uint32_t)(d0 + (((int32_t)d1 - (int32_t)d0) * rF) / 1024);
        if (batt10 < 100) baseDwell += 400;
        else if (batt10 < 115) baseDwell += 200;
        cmd.finalDwellUs = baseDwell;
    } else {
        if      (batt10 < 100) cmd.finalDwellUs = 4500;
        else if (batt10 < 115) cmd.finalDwellUs = 3800;
        else if (batt10 < 125) cmd.finalDwellUs = 3200;
        else if (batt10 < 135) cmd.finalDwellUs = 2800;
        else if (batt10 < 145) cmd.finalDwellUs = 2500;
        else                   cmd.finalDwellUs = 2200;
    }

    if (cmd.finalDwellUs > MAX_DWELL_US) {
        cmd.finalDwellUs = MAX_DWELL_US;
    }

    // Limp-home konservatif (12.0 deg) agar aman dari knocking pada suhu tinggi/sensor error
    if (fault || state == STATE_LIMP_HOME) {
        cmd.finalAdvance10 = 120;
        return cmd;
    }
    if (state == STATE_CRANKING) {
        cmd.finalAdvance10 = 50;
        return cmd;
    }

    int16_t q11, q21, q12, q22;
    if (mode == MODE_DAILY) {
        q11 = mapDaily3DBase[r0][t0]; q21 = mapDaily3DBase[r1][t0];
        q12 = mapDaily3DBase[r0][t1]; q22 = mapDaily3DBase[r1][t1];
    } else if (mode == MODE_RACING) {
        q11 = mapRacing3DBase[r0][t0]; q21 = mapRacing3DBase[r1][t0];
        q12 = mapRacing3DBase[r0][t1]; q22 = mapRacing3DBase[r1][t1];
    } else if (mode == MODE_EXTREME) {
        q11 = mapExtreme3DBase[r0][t0]; q21 = mapExtreme3DBase[r1][t0];
        q12 = mapExtreme3DBase[r0][t1]; q22 = mapExtreme3DBase[r1][t1];
    } else {
        uint8_t bufIdx = atomic_load(&activeMapIndex);
        q11 = activeCustomMapBuffer[bufIdx][r0][t0]; q21 = activeCustomMapBuffer[bufIdx][r1][t0];
        q12 = activeCustomMapBuffer[bufIdx][r0][t1]; q22 = activeCustomMapBuffer[bufIdx][r1][t1];
    }

    int32_t tF = ((int32_t)(calcTps - (t0 * 25)) * 1024) / 25;
    int32_t R1 = q11 + ((q21 - q11) * rF) / 1024;
    int32_t R2 = q12 + ((q22 - q12) * rF) / 1024;

    int16_t adv = (int16_t)(R1 + ((R2 - R1) * tF) / 1024);

    if (fuel == BBM_PERTAMAX && calcRpm >= 4000 && calcTps >= 50) {
        adv += 5;
    }

    if (temp10 > 900) {
        int16_t retard = (int16_t)mapRange(temp10, 900, 1300, 0, 50);
        adv -= retard;
        if (adv < 50) adv = 50;
    }

    cmd.finalAdvance10 = (int16_t)CLAMP(adv, 0, ROTOR_PULSER_DEGREES_10);
    return cmd;
}

static bool IRAM_ATTR onIgnitionTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  uint8_t st = atomic_load(&ignState);
  if (st == IGN_WAITING_CHARGE) {
    TCI_COIL_CHARGE();
    atomic_store(&charge_start_time_us, (uint32_t)esp_timer_get_time());
    atomic_store(&ignState, IGN_WAITING_SPARK);

    uint64_t now_cnt = edata->count_value;
    uint64_t spark_cnt = atomic_load(&target_spark_count);

    if (spark_cnt <= now_cnt + 25) {
        TCI_COIL_SPARK();
        atomic_store(&ignState, IGN_IDLE);
    } else {
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = spark_cnt,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = false
        };
        if (gptimer_set_alarm_action(timer, &alarm_config) != ESP_OK) {
            TCI_COIL_SPARK();
            atomic_store(&ignState, IGN_IDLE);
        }
    }
  } else if (st == IGN_WAITING_SPARK) {
    TCI_COIL_SPARK();
    atomic_store(&ignState, IGN_IDLE);
  }
  return false;
}

static void IRAM_ATTR pulserISR(void* arg) {
  uint32_t now_us = (uint32_t)(esp_timer_get_time());
  uint32_t last_us = atomic_load(&last_pulse_time_us);
  uint32_t interval_us = now_us - last_us;

  // Abaikan glitch noise (< 2000 us -> > 30.000 RPM)
  if (last_us != 0 && interval_us < 2000) return;

  // Jika jeda melebihi timeout mesin mati (stall/re-crank), reset referensi interval agar tidak memicu false accel_delta
  bool freshStart = (last_us == 0) || (interval_us > ENGINE_STALL_TIMEOUT_US);
  if (freshStart) {
      atomic_store(&last_pulse_time_us, now_us);
      atomic_store(&pulse_interval_us, 0);
      atomic_store(&currentRPM, 0);
      SecureData_Write(&sec_engine_state, (uint32_t)STATE_CRANKING);
      TCI_COIL_SPARK();
      atomic_store(&ignState, IGN_IDLE);
      return;
  }

  uint32_t expected_interval = atomic_load(&pulse_interval_us);

  if (expected_interval > 0) {
    // Filter noise pulser palsu di tengah putaran
    if (interval_us < ((expected_interval * 4) / 10)) return;
    if (interval_us > (expected_interval * 3)) {
        if (atomic_load(&ignState) != IGN_IDLE) {
            TCI_COIL_SPARK();
            atomic_store(&ignState, IGN_IDLE);
        }
    }
  }

  int32_t accel_delta_us = (expected_interval > 0) ? ((int32_t)expected_interval - (int32_t)interval_us) : 0;

  uint32_t rpm = 60000000UL / interval_us;
  EngineState newState = (atomic_load(&sensorFaultStatus)) ? STATE_LIMP_HOME : ((rpm < 600) ? STATE_CRANKING : STATE_RUNNING);

  atomic_store(&pulse_interval_us, interval_us);
  atomic_store(&last_pulse_time_us, now_us);
  atomic_store(&currentRPM, (uint16_t)rpm);
  SecureData_Write(&sec_engine_state, (uint32_t)newState);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (ignCalcTaskHandle != NULL) {
      vTaskNotifyGiveFromISR(ignCalcTaskHandle, &xHigherPriorityTaskWoken);
  }

  int16_t adv10 = atomic_load(&cachedAdv10);
  if (newState == STATE_CRANKING || rpm < 600) {
      adv10 = 50;
  } else if (newState == STATE_LIMP_HOME || atomic_load(&sensorFaultStatus)) {
      adv10 = 120;
  }

  uint32_t safeMode = MODE_DAILY;
  uint16_t currentLimit = DEFAULT_DAILY_RPM_LIMIT;
  if (SecureData_Read(&sec_mode, &safeMode)) {
      if (safeMode == MODE_CUSTOM) {
          currentLimit = atomic_load(&activeCustomRpmLimit);
          if (currentLimit < 1000) currentLimit = 1000;
      } else if (safeMode == MODE_RACING) {
          currentLimit = 12200;
      } else if (safeMode == MODE_EXTREME) {
          currentLimit = 12800;
      }
  }
  uint16_t softStartLimit = (currentLimit > 400) ? (uint16_t)(currentLimit - 400) : currentLimit;

  if (newState == STATE_LIMP_HOME && rpm >= 6000) {
      uint8_t lc = atomic_fetch_add(&limp_cut_counter, 1) + 1;
      if (lc % 2 != 0) {
          TCI_COIL_SPARK();
          atomic_store(&ignState, IGN_IDLE);
          if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
          return;
      }
  } else {
      atomic_store(&limp_cut_counter, 0);
  }

  // Soft-retard progresif 400 RPM sebelum hard-cut agar tidak menyentak saat pemakaian harian
  if (rpm >= softStartLimit && rpm < currentLimit && newState == STATE_RUNNING) {
      uint32_t span = (uint32_t)(currentLimit - softStartLimit);
      uint32_t over = (uint32_t)(rpm - softStartLimit);
      int16_t targetSoftAdv = (int16_t)(adv10 - (((int32_t)(adv10 - 60) * (int32_t)over) / (int32_t)span));
      if (targetSoftAdv < adv10) adv10 = targetSoftAdv;
  }

  if (rpm >= currentLimit) {
      uint8_t rc = atomic_fetch_add(&rev_cut_counter, 1) + 1;
      if (rc % 3 != 0) {
          TCI_COIL_SPARK();
          atomic_store(&ignState, IGN_IDLE);
          if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
          return;
      }
      adv10 = 50;
  } else {
      atomic_store(&rev_cut_counter, 0);
  }

  atomic_store(&currentDegree10, adv10);

  uint32_t targetDwellUs = atomic_load(&cachedDwellUs);
  uint32_t maxAllowedDwell = (interval_us > 1200) ? (interval_us - 800) : ((interval_us * 80) / 100);
  if (targetDwellUs > maxAllowedDwell) targetDwellUs = maxAllowedDwell;

  int32_t sparkDegFromPulser = ROTOR_PULSER_DEGREES_10 - adv10;
  if (sparkDegFromPulser < 0) sparkDegFromPulser = 0;

  uint32_t sparkDelayUs = (uint32_t)(((uint64_t)sparkDegFromPulser * (uint64_t)interval_us * INV_3600_Q32) >> 32);

  if (accel_delta_us > 0 && sparkDelayUs > 0 && interval_us > 0) {
      uint32_t accel_comp = (sparkDelayUs * (uint32_t)accel_delta_us) / (4 * interval_us);
      uint32_t max_comp = sparkDelayUs / 6;
      if (accel_comp > max_comp) accel_comp = max_comp;
      sparkDelayUs -= accel_comp;
  }

  uint64_t current_count = 0;
  if (gptimer_get_raw_count(timerIgnition, &current_count) != ESP_OK) {
      TCI_COIL_SPARK();
      atomic_store(&ignState, IGN_IDLE);
      if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
      return;
  }

  if (sparkDelayUs >= interval_us || sparkDelayUs < 30) {
    TCI_COIL_SPARK();
    atomic_store(&ignState, IGN_IDLE);
  } else {
    uint32_t effDwellUs = targetDwellUs;
    if (sparkDelayUs <= 350) {
        effDwellUs = sparkDelayUs / 2;
    } else if (effDwellUs > (sparkDelayUs - 150)) {
        effDwellUs = sparkDelayUs - 150;
    }

    uint64_t spark_count = current_count + sparkDelayUs;
    uint64_t charge_count = (effDwellUs >= 50) ? (spark_count - effDwellUs) : (current_count + 15);

    atomic_store(&target_spark_count, spark_count);
    atomic_store(&ignState, IGN_WAITING_CHARGE);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = charge_count,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false
    };
    if (gptimer_set_alarm_action(timerIgnition, &alarm_config) != ESP_OK) {
        TCI_COIL_SPARK();
        atomic_store(&ignState, IGN_IDLE);
    }
  }

  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void codeTaskIgnitionCalc(void * parameter) {
  esp_task_wdt_add(NULL);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    esp_task_wdt_reset();

    uint32_t now = (uint32_t)esp_timer_get_time();
    uint32_t start_us = atomic_load(&charge_start_time_us);
    if (atomic_load(&ignState) == IGN_WAITING_SPARK && (now - start_us) > 7000UL) {
        TCI_COIL_SPARK();
        atomic_store(&ignState, IGN_IDLE);
    }

    uint16_t rpm = atomic_load(&currentRPM);
    uint8_t tps = atomic_load(&currentTPS);
    int16_t temp = atomic_load(&currentEngineTemp10);
    uint16_t batt = atomic_load(&currentBatteryVoltage10);
    uint8_t fuel = atomic_load(&currentFuel);
    bool fault = atomic_load(&sensorFaultStatus);

    uint32_t safeMode, safeState;
    if (!SecureData_Read(&sec_mode, &safeMode) || !SecureData_Read(&sec_engine_state, &safeState)) {
        safeMode = (uint32_t)MODE_DAILY;
        safeState = (uint32_t)STATE_LIMP_HOME;
    }

    IgnitionCommand_t cmd = HAL_IgnitionLogic_Calculate(rpm, tps, temp, batt, fuel, (ModePengapian)safeMode, (EngineState)safeState, fault);

    atomic_store(&cachedAdv10, cmd.finalAdvance10);
    atomic_store(&cachedDwellUs, cmd.finalDwellUs);
  }
}

#define MAX_TPS_GRADIENT_PER_10MS  15
#define MAX_TEMP_GRADIENT_PER_10MS 50

void codeTaskSensor(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint16_t emaTps = 0, emaBatt = 0, emaTemp = 0;
  static uint8_t lastValidTps = 0;
  static int16_t lastValidTemp = 300;
  static uint8_t faultDebounceCnt = 0;

  for(;;) {
    esp_task_wdt_reset();
    int rawTps = 0, rawBatt = 0, rawTemp = 0;
    int mvTps = 0, mvBatt = 0, mvTemp = 0;
    bool adcReadOk = true;

    if (adc1_handle) {
        int sumTps = 0, sumTemp = 0, sumBatt = 0, tmpSample = 0;
        for (int s = 0; s < 4 && adcReadOk; s++) {
            if (adc_oneshot_read(adc1_handle, ADC_CHAN_TPS, &tmpSample) == ESP_OK) sumTps += tmpSample;
            else adcReadOk = false;

            if (adc_oneshot_read(adc1_handle, ADC_CHAN_TEMP, &tmpSample) == ESP_OK) sumTemp += tmpSample;
            else adcReadOk = false;

            if (adc_oneshot_read(adc1_handle, ADC_CHAN_BATT, &tmpSample) == ESP_OK) sumBatt += tmpSample;
            else adcReadOk = false;
        }
        if (adcReadOk) {
            rawTps = sumTps >> 2;
            rawTemp = sumTemp >> 2;
            rawBatt = sumBatt >> 2;

            if (adc1_cali_handle) {
                adc_cali_raw_to_voltage(adc1_cali_handle, rawTps, &mvTps);
                adc_cali_raw_to_voltage(adc1_cali_handle, rawTemp, &mvTemp);
                adc_cali_raw_to_voltage(adc1_cali_handle, rawBatt, &mvBatt);
            } else {
                mvTps = adc_raw_to_mv_fallback(rawTps);
                mvTemp = adc_raw_to_mv_fallback(rawTemp);
                mvBatt = adc_raw_to_mv_fallback(rawBatt);
            }
        }
    } else {
        adcReadOk = false;
    }

    if (!adcReadOk) {
        setFaultBit(FAULT_ADC_ERR);
    } else {
        clearFaultBit(FAULT_ADC_ERR);

        if (rawTps < 40 || rawTps > 4060) setFaultBit(FAULT_TPS_OUT);
        else clearFaultBit(FAULT_TPS_OUT);

        if (rawTemp < 25 || rawTemp > 4060) setFaultBit(FAULT_NTC_OUT);
        else clearFaultBit(FAULT_NTC_OUT);

        if (mvBatt < 550) {
            setFaultBit(FAULT_BATT_LOW);
            clearFaultBit(FAULT_BATT_HIGH);
        } else if (mvBatt > 3200) {
            setFaultBit(FAULT_BATT_HIGH);
            clearFaultBit(FAULT_BATT_LOW);
        } else {
            clearFaultBit(FAULT_BATT_LOW | FAULT_BATT_HIGH);
        }

        emaTps  = (emaTps == 0)  ? (uint16_t)mvTps  : (uint16_t)(emaTps - (emaTps >> 2) + (mvTps >> 2));
        emaTemp = (emaTemp == 0) ? (uint16_t)mvTemp : (uint16_t)(emaTemp - (emaTemp >> 2) + (mvTemp >> 2));
        emaBatt = (emaBatt == 0) ? (uint16_t)mvBatt : (uint16_t)(emaBatt - (emaBatt >> 2) + (mvBatt >> 2));

        int32_t target_tps   = mapRange(emaTps, 450, 2800, 0, 100);
        int32_t target_tempC = mapRange(emaTemp, 300, 2800, -200, 1500);
        uint16_t battV       = (uint16_t)mapRange(emaBatt, 0, 3250, 0, 160);

        int32_t tpsP_raw = CLAMP(target_tps, (int32_t)lastValidTps - MAX_TPS_GRADIENT_PER_10MS, (int32_t)lastValidTps + MAX_TPS_GRADIENT_PER_10MS);
        uint8_t tpsP = (uint8_t)CLAMP(tpsP_raw, 0, 100);
        lastValidTps = tpsP;

        int32_t tempC_raw = CLAMP(target_tempC, (int32_t)lastValidTemp - MAX_TEMP_GRADIENT_PER_10MS, (int32_t)lastValidTemp + MAX_TEMP_GRADIENT_PER_10MS);
        int16_t tempC = (int16_t)CLAMP(tempC_raw, -400, 2000);
        lastValidTemp = tempC;

        atomic_store(&currentTPS, tpsP);
        atomic_store(&currentEngineTemp10, tempC);
        atomic_store(&currentBatteryVoltage10, battV);
    }

    // Debounce fault sensor 50ms (5 siklus x 10ms) agar glitch EMI busi sesaat tidak langsung memicu limp-home
    uint16_t currentFaults = atomic_load(&hardwareFaultCode);
    bool rawFault = ((currentFaults & SENSOR_FAULT_MASK) != FAULT_NONE);
    if (rawFault) {
        if (faultDebounceCnt < 5) faultDebounceCnt++;
        if (faultDebounceCnt >= 5) atomic_store(&sensorFaultStatus, true);
    } else {
        faultDebounceCnt = 0;
        atomic_store(&sensorFaultStatus, false);
    }

    uint32_t now = (uint32_t)esp_timer_get_time();
    uint32_t start_us = atomic_load(&charge_start_time_us);
    if (atomic_load(&ignState) != IGN_IDLE && (now - start_us) > 7000UL) {
        TCI_COIL_SPARK();
        atomic_store(&ignState, IGN_IDLE);
    }

    uint32_t last_pulse = atomic_load(&last_pulse_time_us);
    if (last_pulse > 0 && (now - last_pulse) > ENGINE_STALL_TIMEOUT_US) {
      if (atomic_load(&currentRPM) != 0) {
          atomic_store(&currentRPM, 0);
          atomic_store(&pulse_interval_us, 0);
          atomic_store(&ignState, IGN_IDLE);
          TCI_COIL_SPARK();
      }
      atomic_store(&limp_cut_counter, 0);
      atomic_store(&rev_cut_counter, 0);
      SecureData_Write(&sec_engine_state, (uint32_t)STATE_STOPPED);
    }

    // Deferred NVS save otomatis segera setelah mesin berhenti
    if (atomic_load(&pendingModeSaveToNvs) && isEngineStopped()) {
      uint8_t modeToSave = atomic_load(&pendingModeValue);
      nvs_save_mode(modeToSave);
      atomic_store(&pendingModeSaveToNvs, false);
    }
    if (atomic_load(&pendingSlotSaveToNvs) && isEngineStopped()) {
      uint8_t slotToSave = atomic_load(&pendingSlotValue);
      nvs_save_custom_slot(slotToSave);
      atomic_store(&pendingSlotSaveToNvs, false);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void codeTaskTachOutput(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint32_t lastFreqHz = 0;
  for (;;) {
    esp_task_wdt_reset();
    uint16_t rpm = atomic_load(&currentRPM);
    if (rpm > 400) {
      uint32_t freqHz = rpm / 60;
      if (freqHz < 1) freqHz = 1;
      if (freqHz != lastFreqHz) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freqHz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        lastFreqHz = freqHz;
      }
    } else {
      if (lastFreqHz != 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        lastFreqHz = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void codeTaskMemoryScrubber(void * parameter) {
    esp_task_wdt_add(NULL);
    for(;;) {
        esp_task_wdt_reset();
        uint32_t dummy;
        if (!SecureData_Read(&sec_mode, &dummy)) SecureData_Write(&sec_mode, (uint32_t)MODE_DAILY);
        if (!SecureData_Read(&sec_engine_state, &dummy)) SecureData_Write(&sec_engine_state, (uint32_t)STATE_STOPPED);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void codeTaskTelemetry(void * parameter) {
  esp_task_wdt_add(NULL);
  for(;;) {
    esp_task_wdt_reset();
    TelemetryData pkt;
    pkt.header = 0xAA55;
    pkt.rpm = atomic_load(&currentRPM);
    pkt.tps = atomic_load(&currentTPS);
    pkt.degree10 = atomic_load(&currentDegree10);
    pkt.temp10 = atomic_load(&currentEngineTemp10);
    pkt.battVolt10 = atomic_load(&currentBatteryVoltage10);

    uint32_t m, s;
    SecureData_Read(&sec_mode, &m);
    SecureData_Read(&sec_engine_state, &s);
    pkt.mode = (uint8_t)m;
    pkt.state = (uint8_t)s;
    pkt.fuel = atomic_load(&currentFuel);
    pkt.ackStatus = atomic_load(&lastAckStatus);
    pkt.faultCode = atomic_load(&hardwareFaultCode);

    pkt.crc16 = calculateCRC16((uint8_t*)&pkt, sizeof(TelemetryData) - sizeof(uint16_t));

    size_t free_tx = 0;
    static uint8_t ackHoldCount = 0;
    if (uart_get_tx_buffer_free_size(UART_NUM_2, &free_tx) == ESP_OK && free_tx >= sizeof(TelemetryData)) {
        if (uart_write_bytes(UART_NUM_2, (const char*)&pkt, sizeof(TelemetryData)) == (int)sizeof(TelemetryData)) {
            if (pkt.ackStatus != 0) {
                ackHoldCount++;
                if (ackHoldCount >= 5) {
                    atomic_store(&lastAckStatus, 0);
                    ackHoldCount = 0;
                }
            } else {
                ackHoldCount = 0;
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void codeTaskCommandListener(void * parameter) {
  esp_task_wdt_add(NULL);
  static uint8_t rxBuf[sizeof(CommandPacket)];
  static uint16_t rxIdx = 0;
  static uint8_t syncState = 0;
  static TickType_t lastByteTick = 0;

  for(;;) {
    esp_task_wdt_reset();

    if (syncState != 0 && (xTaskGetTickCount() - lastByteTick) > pdMS_TO_TICKS(60)) {
        syncState = 0;
        rxIdx = 0;
    }

    uint8_t byte;
    while (uart_read_bytes(UART_NUM_2, &byte, 1, pdMS_TO_TICKS(5)) > 0) {
      lastByteTick = xTaskGetTickCount();
      switch (syncState) {
        case 0:
          if (byte == 0x55) { rxBuf[0] = byte; rxIdx = 1; syncState = 1; }
          break;
        case 1:
          if (byte == 0xCC) { rxBuf[1] = byte; rxIdx = 2; syncState = 2; }
          else if (byte == 0x55) { rxBuf[0] = byte; rxIdx = 1; }
          else { syncState = 0; rxIdx = 0; }
          break;
        case 2:
          rxBuf[rxIdx++] = byte;
          if (rxIdx >= sizeof(CommandPacket)) {
            CommandPacket *cmd = (CommandPacket*)rxBuf;
            uint16_t calcCrc = calculateCRC16(rxBuf, sizeof(CommandPacket) - sizeof(uint16_t));

            if (cmd->crc16 == calcCrc) {
              if (cmd->cmdType == 0x01) {
                if (cmd->slotOrMode <= MODE_CUSTOM) {
                  if ((ModePengapian)cmd->slotOrMode == MODE_CUSTOM) {
                    updateActiveMapBuffer();
                  }
                  SecureData_Write(&sec_mode, (uint32_t)cmd->slotOrMode);
                  if (isEngineStopped()) {
                    nvs_save_mode((uint8_t)cmd->slotOrMode);
                    atomic_store(&pendingModeSaveToNvs, false);
                    atomic_store(&lastAckStatus, 1);
                  } else {
                    atomic_store(&pendingModeValue, (uint8_t)cmd->slotOrMode);
                    atomic_store(&pendingModeSaveToNvs, true);
                    atomic_store(&lastAckStatus, 3);
                  }
                } else {
                  atomic_store(&lastAckStatus, 2);
                }
              }
              else if (cmd->cmdType == 0x02 || cmd->cmdType == 0x03 || cmd->cmdType == 0x04) {
                uint8_t slot = cmd->slotOrMode;
                if (slot < MAX_CUSTOM_SLOTS) {
                  uint16_t safeLimit = CLAMP(cmd->rpmLimit, 1000, 15000);
                  portENTER_CRITICAL_SAFE(&secure_mux);
                  for (int r = 0; r < NUM_RPM_POINTS; r++) {
                      for (int t = 0; t < NUM_TPS_POINTS; t++) {
                          mapCustomSlots[slot][r][t] = (int16_t)CLAMP(cmd->mapData[r][t], 0, ROTOR_PULSER_DEGREES_10);
                      }
                      dwellCustomSlots[slot][r] = (uint16_t)CLAMP(cmd->dwellData[r], 1000, MAX_DWELL_US);
                  }
                  rpmLimitCustomSlots[slot] = safeLimit;
                  portEXIT_CRITICAL_SAFE(&secure_mux);

                  if (cmd->cmdType == 0x04) {
                    if (atomic_load(&currentCustomSlot) == slot) {
                      updateActiveMapBuffer();
                    }
                    atomic_store(&lastAckStatus, 4);
                  } else {
                    atomic_store(&currentCustomSlot, slot);
                    updateActiveMapBuffer();

                    if (cmd->cmdType == 0x03) {
                      if (isEngineStopped()) {
                        nvs_save_custom_slot(slot);
                        atomic_store(&pendingSlotSaveToNvs, false);
                        atomic_store(&lastAckStatus, 1);
                      } else {
                        atomic_store(&pendingSlotValue, slot);
                        atomic_store(&pendingSlotSaveToNvs, true);
                        atomic_store(&lastAckStatus, 3);
                      }
                    } else {
                      atomic_store(&lastAckStatus, 1);
                    }
                  }
                } else {
                  atomic_store(&lastAckStatus, 2);
                }
              } else if (cmd->cmdType == 0x05) {
                if (cmd->slotOrMode <= BBM_PERTAMAX) {
                  atomic_store(&currentFuel, cmd->slotOrMode);
                  atomic_store(&lastAckStatus, 1);
                } else {
                  atomic_store(&lastAckStatus, 2);
                }
              } else {
                atomic_store(&lastAckStatus, 2);
              }
            } else {
              atomic_store(&lastAckStatus, 2);
            }
            syncState = 0;
            rxIdx = 0;
          }
          break;
      }
    }
  }
}

#define STACK_SIZE_IGN   4096
#define STACK_SIZE_SENS  4096
#define STACK_SIZE_TACH  2048
#define STACK_SIZE_SCRUB 2048
#define STACK_SIZE_TEL   3072
#define STACK_SIZE_CMD   4096

static StackType_t ignTaskStack[STACK_SIZE_IGN];
static StaticTask_t ignTaskBuffer;
static StackType_t sensTaskStack[STACK_SIZE_SENS];
static StaticTask_t sensTaskBuffer;
static StackType_t tachTaskStack[STACK_SIZE_TACH];
static StaticTask_t tachTaskBuffer;
static StackType_t scrubTaskStack[STACK_SIZE_SCRUB];
static StaticTask_t scrubTaskBuffer;
static StackType_t telTaskStack[STACK_SIZE_TEL];
static StaticTask_t telTaskBuffer;
static StackType_t cmdTaskStack[STACK_SIZE_CMD];
static StaticTask_t cmdTaskBuffer;

void app_main(void) {
  esp_ota_mark_app_valid_cancel_rollback();

  SecureData_Write(&sec_mode, (uint32_t)MODE_DAILY);
  SecureData_Write(&sec_engine_state, (uint32_t)STATE_STOPPED);

  initNvsAndLoadMaps();

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic = true
  };
  ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

  gpio_config_t io_out = {
    .pin_bit_mask = (1ULL << PIN_TCI),
    .mode = GPIO_MODE_OUTPUT,
    .pull_down_en = 1,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&io_out));
  TCI_COIL_SPARK();

  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 10,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = PIN_TACH_OUT,
    .duty = 0,
    .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  uart_config_t uart_cfg = {
    .baud_rate = 250000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, PIN_TX_TELEMETRY, PIN_RX_TELEMETRY, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0));

  adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_TARGET };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHAN_TPS, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHAN_TEMP, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHAN_BATT, &config));

  adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_TARGET,
    .bitwidth = ADC_BITWIDTH_12,
  };

  if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) != ESP_OK) {
      adc1_cali_handle = NULL;
      ESP_LOGW(TAG, "ADC calibration fallback aktif");
  }

  gptimer_config_t timer_cfg = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,
    .flags.intr_shared = false
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timerIgnition));
  gptimer_event_callbacks_t cbs = { .on_alarm = onIgnitionTimer };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerIgnition, &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(timerIgnition));
  ESP_ERROR_CHECK(gptimer_start(timerIgnition));

  ignCalcTaskHandle = xTaskCreateStaticPinnedToCore(
      codeTaskIgnitionCalc, "TaskIgnCalc", STACK_SIZE_IGN, NULL, 20,
      ignTaskStack, &ignTaskBuffer, 1);

  xTaskCreateStaticPinnedToCore(
      codeTaskSensor, "TaskSens", STACK_SIZE_SENS, NULL, 5,
      sensTaskStack, &sensTaskBuffer, 1);

  xTaskCreateStaticPinnedToCore(
      codeTaskTachOutput, "TaskTach", STACK_SIZE_TACH, NULL, 2,
      tachTaskStack, &tachTaskBuffer, 1);

  xTaskCreateStaticPinnedToCore(
      codeTaskMemoryScrubber, "TaskScrub", STACK_SIZE_SCRUB, NULL, 5,
      scrubTaskStack, &scrubTaskBuffer, 0);

  xTaskCreateStaticPinnedToCore(
      codeTaskTelemetry, "TaskTel", STACK_SIZE_TEL, NULL, 3,
      telTaskStack, &telTaskBuffer, 0);

  xTaskCreateStaticPinnedToCore(
      codeTaskCommandListener, "TaskCmd", STACK_SIZE_CMD, NULL, 2,
      cmdTaskStack, &cmdTaskBuffer, 0);

  gpio_config_t io_puls = {
    .pin_bit_mask = (1ULL << PIN_PULSER),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = 1,
    .intr_type = GPIO_INTR_NEGEDGE
  };
  ESP_ERROR_CHECK(gpio_config(&io_puls));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_PULSER, pulserISR, NULL));

  ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
}
```[cite: 1]

### Ringkasan Perubahan
* **`ADC_CHAN_BATT`:** Diubah dari `ADC_CHANNEL_2` (**GPIO 3**) menjadi `ADC_CHANNEL_3` (**GPIO 4**)[cite: 1].
* **Konfigurasi lainnya:** Seluruh logika pengapian, *timer*, *interrupt pulser*, NVS, *telemetry/command* UART, serta pemetaan ADC (`battVolt10`) tetap dipertahankan[cite: 1].

Apakah Anda ingin saya bantu menghitung nilai resistor *voltage divider* untuk jalur aki ke **GPIO 4**?
