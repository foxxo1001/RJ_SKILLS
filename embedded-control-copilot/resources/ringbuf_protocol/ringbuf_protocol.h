/**
 * @file ringbuf_protocol.h
 * @brief Lock-free SPSC RingBuffer, deterministic UART binary frame parser,
 *        VOFA+ JustFloat telemetry builder, and zero-malloc ASCII live PID tuner.
 *        无锁 SPSC 环形缓冲区 + 抗粘包/断包状态机协议解析器 + VOFA+ 波形打包 + 在线调参解析。
 *
 * Integration pattern across all 4 contest MCUs:
 *   - STM32:   Call ringbuf_push_block() inside HAL_UARTEx_RxEventCallback (DMA Idle).
 *   - MSPM0:   Call ringbuf_push_byte() inside UART_INST_IRQHandler (RX FIFO drain).
 *   - ESP32:   Read uart_read_bytes() in RX task and feed proto_parser_feed_byte().
 *   - Renesas: Call ringbuf_push_byte() inside R_SCI_UART callback on UART_EVENT_RX_CHAR.
 *   Then poll proto_parser_poll_ringbuf() in main loop or soft-RT task!
 */

#ifndef RINGBUF_PROTOCOL_H
#define RINGBUF_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 1. Lock-Free SPSC Ring Buffer ===================== */

typedef struct {
    uint8_t *buf;
    uint32_t capacity;           /**< Buffer size in bytes (usable capacity = capacity - 1) */
    volatile uint32_t head;      /**< Write index (modified ONLY by producer / ISR) */
    volatile uint32_t tail;      /**< Read index  (modified ONLY by consumer / main) */
    volatile uint32_t drop_cnt;  /**< Overflow drop counter for diagnostics */
} ringbuf_t;

bool ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint32_t capacity);
void ringbuf_reset(ringbuf_t *rb);
uint32_t ringbuf_available(const ringbuf_t *rb);
uint32_t ringbuf_free_space(const ringbuf_t *rb);

/** Producer side (ISR / DMA Callback safe) */
bool ringbuf_push_byte(ringbuf_t *rb, uint8_t byte);
uint32_t ringbuf_push_block(ringbuf_t *rb, const uint8_t *data, uint32_t len);

/** Consumer side (Main loop / Task safe) */
bool ringbuf_pop_byte(ringbuf_t *rb, uint8_t *out_byte);
uint32_t ringbuf_pop_block(ringbuf_t *rb, uint8_t *out_data, uint32_t max_len);

/* ===================== 2. Binary Frame Protocol Parser ===================== */
/* Wire format: [0xAA][0x55][CMD:1B][LEN:1B][PAYLOAD:LEN B][CHK:1B]
 * Checksum covers [CMD, LEN, PAYLOAD[0..LEN-1]]. */

#ifndef PROTO_SOF1
#define PROTO_SOF1 (0xAAu)
#endif

#ifndef PROTO_SOF2
#define PROTO_SOF2 (0x55u)
#endif

#ifndef PROTO_MAX_PAYLOAD
#define PROTO_MAX_PAYLOAD (64u)
#endif

typedef enum {
    PROTO_CHK_SUM8 = 0,       /**< Simple 8-bit sum modulo 256 (fastest for OpenMV/MaixPy) */
    PROTO_CHK_CRC8_MAXIM = 1  /**< Polynomial 0x31 reflected (0x8C), strong EMI protection */
} proto_chk_mode_t;

typedef enum {
    PROTO_ST_WAIT_SOF1 = 0,
    PROTO_ST_WAIT_SOF2,
    PROTO_ST_WAIT_CMD,
    PROTO_ST_WAIT_LEN,
    PROTO_ST_WAIT_PAYLOAD,
    PROTO_ST_WAIT_CHK
} proto_state_t;

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

typedef void (*proto_frame_cb_t)(const proto_frame_t *frame, void *user_ctx);

typedef struct {
    proto_chk_mode_t chk_mode;
    proto_state_t state;
    proto_frame_t current;
    uint8_t payload_idx;
    uint8_t running_chk;

    proto_frame_cb_t on_frame;
    void *user_ctx;

    /* Telemetry & Comm Watchdog timestamps */
    uint32_t rx_valid_cnt;
    uint32_t rx_err_cnt;
    uint32_t last_rx_tick_ms;
} proto_parser_t;

uint8_t proto_calc_checksum(proto_chk_mode_t mode, uint8_t cmd, uint8_t len,
                            const uint8_t *payload);

void proto_parser_init(proto_parser_t *p, proto_chk_mode_t chk_mode,
                       proto_frame_cb_t on_frame, void *user_ctx);

/**
 * @brief Feed one byte into the state machine.
 * @return true if a complete, checksum-verified frame was parsed on this byte.
 */
bool proto_parser_feed_byte(proto_parser_t *p, uint8_t byte, uint32_t now_ms);

/**
 * @brief Drain up to max_bytes from a ringbuf into parser (bounds worst-case main-loop time).
 * @return Number of valid frames dispatched.
 */
uint32_t proto_parser_poll_ringbuf(proto_parser_t *p, ringbuf_t *rb, uint32_t now_ms,
                                   uint32_t max_bytes);

/**
 * @brief Pack a binary protocol frame into out_buf.
 * @return Total frame length in bytes (5 + len), or 0 on error/insufficient buffer.
 */
size_t proto_pack_frame(proto_chk_mode_t mode, uint8_t cmd, const uint8_t *payload, uint8_t len,
                        uint8_t *out_buf, size_t out_buf_size);

/* ===================== 3. VOFA+ JustFloat & Live ASCII Tuner ===================== */

/**
 * @brief Pack N float channels into VOFA+ JustFloat binary format (little-endian + 00 00 80 7F).
 *        Uses memcpy to guarantee alignment safety on Cortex-M0+ (TI MSPM0).
 * @return Byte count written (ch_cnt * 4 + 4), or 0 if buffer too small.
 */
size_t vofa_pack_justfloat(const float *channels, uint8_t ch_cnt, uint8_t *out_buf,
                           size_t out_buf_size);

typedef struct {
    const char *name;  /**< Case-insensitive ASCII key, e.g. "KP", "KI", "KD", "TAR" */
    float *target_ptr; /**< Pointer to float variable to update live */
    float min_val;
    float max_val;
} tuner_param_t;

/**
 * @brief Parse one ASCII tuning command like "KP=1.25\r\n" or "TAR=-30.5" without sscanf/malloc.
 * @return Index (>= 0) of updated parameter in table, or -1 if unmatched/invalid.
 */
int32_t tuner_parse_line(const char *line, const tuner_param_t *table, size_t count);

#ifdef __cplusplus
}
#endif

#endif /* RINGBUF_PROTOCOL_H */
