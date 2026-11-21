#ifndef COMMON_PROTO_H
#define COMMON_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// KONSTANTA PROTOKOL & IDENTITAS PERANGKAT SATRIA FU TCI SYSTEM
// ============================================================================
#define PROTO_HEADER_TELEMETRY   0xAA55U
#define PROTO_HEADER_HUD_CMD     0xCC55U
#define PROTO_HEADER_ECU_CMD     0x55CCU

#define PROTO_NUM_RPM_POINTS     15U
#define PROTO_NUM_TPS_POINTS     5U
#define PROTO_MAX_CUSTOM_SLOTS   5U

#define PROTO_UART_BAUDRATE      250000UL
#define PROTO_ESPNOW_CHANNEL     1U

// MAC Address Default Perangkat (Locally Administered Unicasts)
static const uint8_t PROTO_MAC_C3_TELEMETRY[6] = {0x02, 0x53, 0x46, 0x55, 0x43, 0x33};
static const uint8_t PROTO_MAC_S3_HUD[6]       = {0x02, 0x53, 0x46, 0x55, 0x48, 0x55};

// ============================================================================
// FUNGSI HELPER BERSAMA (C & C++ / ESP-IDF & ARDUINO COMPATIBLE)
// ============================================================================
static inline bool protoMacEquals(const uint8_t *macA, const uint8_t *macB) {
    if (macA == NULL || macB == NULL) {
        return false;
    }
    return (memcmp(macA, macB, 6) == 0);
}

static inline uint16_t protoCalculateCRC16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    if (data == NULL) {
        return crc;
    }
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif // COMMON_PROTO_H
