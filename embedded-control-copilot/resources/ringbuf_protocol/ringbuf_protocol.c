/**
 * @file ringbuf_protocol.c
 * @brief Lock-free SPSC RingBuffer, UART binary frame parser, VOFA+ JustFloat, and ASCII tuner.
 */

#include "ringbuf_protocol.h"

#include <math.h>
#include <string.h>

/* ===================== 1. Lock-Free SPSC Ring Buffer ===================== */

static uint32_t rb_next_idx(uint32_t idx, uint32_t cap) {
    uint32_t next = idx + 1u;
    return (next >= cap) ? 0u : next;
}

bool ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint32_t capacity) {
    if ((rb == NULL) || (storage == NULL) || (capacity < 2u)) {
        return false;
    }
    rb->buf = storage;
    rb->capacity = capacity;
    rb->head = 0u;
    rb->tail = 0u;
    rb->drop_cnt = 0u;
    return true;
}

void ringbuf_reset(ringbuf_t *rb) {
    if (rb == NULL) {
        return;
    }
    rb->head = 0u;
    rb->tail = 0u;
}

uint32_t ringbuf_available(const ringbuf_t *rb) {
    if ((rb == NULL) || (rb->capacity < 2u)) {
        return 0u;
    }
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    if (head >= tail) {
        return head - tail;
    }
    return (rb->capacity - tail) + head;
}

uint32_t ringbuf_free_space(const ringbuf_t *rb) {
    if ((rb == NULL) || (rb->capacity < 2u)) {
        return 0u;
    }
    return (rb->capacity - 1u) - ringbuf_available(rb);
}

bool ringbuf_push_byte(ringbuf_t *rb, uint8_t byte) {
    if ((rb == NULL) || (rb->buf == NULL) || (rb->capacity < 2u)) {
        return false;
    }
    uint32_t head = rb->head;
    uint32_t next = rb_next_idx(head, rb->capacity);
    if (next == rb->tail) {
        rb->drop_cnt++;
        return false;
    }
    rb->buf[head] = byte;
    rb->head = next;
    return true;
}

uint32_t ringbuf_push_block(ringbuf_t *rb, const uint8_t *data, uint32_t len) {
    if ((rb == NULL) || (data == NULL) || (len == 0u)) {
        return 0u;
    }
    uint32_t pushed = 0u;
    for (uint32_t i = 0u; i < len; i++) {
        if (!ringbuf_push_byte(rb, data[i])) {
            rb->drop_cnt += (len - i - 1u);
            break;
        }
        pushed++;
    }
    return pushed;
}

bool ringbuf_pop_byte(ringbuf_t *rb, uint8_t *out_byte) {
    if ((rb == NULL) || (rb->buf == NULL) || (out_byte == NULL) || (rb->capacity < 2u)) {
        return false;
    }
    uint32_t tail = rb->tail;
    if (tail == rb->head) {
        return false;
    }
    *out_byte = rb->buf[tail];
    rb->tail = rb_next_idx(tail, rb->capacity);
    return true;
}

uint32_t ringbuf_pop_block(ringbuf_t *rb, uint8_t *out_data, uint32_t max_len) {
    if ((rb == NULL) || (out_data == NULL) || (max_len == 0u)) {
        return 0u;
    }
    uint32_t popped = 0u;
    while (popped < max_len) {
        if (!ringbuf_pop_byte(rb, &out_data[popped])) {
            break;
        }
        popped++;
    }
    return popped;
}

/* ===================== 2. Binary Frame Protocol Parser ===================== */

static uint8_t crc8_maxim_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0u; i < 8u; i++) {
        if ((crc & 0x01u) != 0u) {
            crc = (uint8_t)((crc >> 1u) ^ 0x8Cu);
        } else {
            crc = (uint8_t)(crc >> 1u);
        }
    }
    return crc;
}

static uint8_t chk_update(proto_chk_mode_t mode, uint8_t acc, uint8_t data) {
    if (mode == PROTO_CHK_CRC8_MAXIM) {
        return crc8_maxim_update(acc, data);
    }
    return (uint8_t)(acc + data);
}

uint8_t proto_calc_checksum(proto_chk_mode_t mode, uint8_t cmd, uint8_t len,
                            const uint8_t *payload) {
    uint8_t acc = 0u;
    acc = chk_update(mode, acc, cmd);
    acc = chk_update(mode, acc, len);
    if (payload != NULL) {
        for (uint8_t i = 0u; i < len; i++) {
            acc = chk_update(mode, acc, payload[i]);
        }
    }
    return acc;
}

void proto_parser_init(proto_parser_t *p, proto_chk_mode_t chk_mode,
                       proto_frame_cb_t on_frame, void *user_ctx) {
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->chk_mode = chk_mode;
    p->state = PROTO_ST_WAIT_SOF1;
    p->on_frame = on_frame;
    p->user_ctx = user_ctx;
}

bool proto_parser_feed_byte(proto_parser_t *p, uint8_t byte, uint32_t now_ms) {
    if (p == NULL) {
        return false;
    }

    switch (p->state) {
        case PROTO_ST_WAIT_SOF1:
            if (byte == PROTO_SOF1) {
                p->state = PROTO_ST_WAIT_SOF2;
            }
            break;

        case PROTO_ST_WAIT_SOF2:
            if (byte == PROTO_SOF2) {
                p->state = PROTO_ST_WAIT_CMD;
            } else if (byte == PROTO_SOF1) {
                /* Handles 0xAA 0xAA 0x55 seamlessly without losing sync! */
                p->state = PROTO_ST_WAIT_SOF2;
            } else {
                p->state = PROTO_ST_WAIT_SOF1;
            }
            break;

        case PROTO_ST_WAIT_CMD:
            p->current.cmd = byte;
            p->running_chk = chk_update(p->chk_mode, 0u, byte);
            p->state = PROTO_ST_WAIT_LEN;
            break;

        case PROTO_ST_WAIT_LEN:
            if (byte > PROTO_MAX_PAYLOAD) {
                p->rx_err_cnt++;
                p->state = (byte == PROTO_SOF1) ? PROTO_ST_WAIT_SOF2 : PROTO_ST_WAIT_SOF1;
            } else {
                p->current.len = byte;
                p->running_chk = chk_update(p->chk_mode, p->running_chk, byte);
                p->payload_idx = 0u;
                p->state = (byte == 0u) ? PROTO_ST_WAIT_CHK : PROTO_ST_WAIT_PAYLOAD;
            }
            break;

        case PROTO_ST_WAIT_PAYLOAD:
            p->current.payload[p->payload_idx++] = byte;
            p->running_chk = chk_update(p->chk_mode, p->running_chk, byte);
            if (p->payload_idx >= p->current.len) {
                p->state = PROTO_ST_WAIT_CHK;
            }
            break;

        case PROTO_ST_WAIT_CHK:
            p->state = PROTO_ST_WAIT_SOF1;
            if (byte == p->running_chk) {
                p->rx_valid_cnt++;
                p->last_rx_tick_ms = now_ms;
                if (p->on_frame != NULL) {
                    p->on_frame(&p->current, p->user_ctx);
                }
                return true;
            }
            p->rx_err_cnt++;
            if (byte == PROTO_SOF1) {
                p->state = PROTO_ST_WAIT_SOF2;
            }
            break;

        default:
            p->state = PROTO_ST_WAIT_SOF1;
            break;
    }

    return false;
}

uint32_t proto_parser_poll_ringbuf(proto_parser_t *p, ringbuf_t *rb, uint32_t now_ms,
                                   uint32_t max_bytes) {
    if ((p == NULL) || (rb == NULL) || (max_bytes == 0u)) {
        return 0u;
    }
    uint32_t frames = 0u;
    uint8_t byte = 0u;
    for (uint32_t i = 0u; i < max_bytes; i++) {
        if (!ringbuf_pop_byte(rb, &byte)) {
            break;
        }
        if (proto_parser_feed_byte(p, byte, now_ms)) {
            frames++;
        }
    }
    return frames;
}

size_t proto_pack_frame(proto_chk_mode_t mode, uint8_t cmd, const uint8_t *payload, uint8_t len,
                        uint8_t *out_buf, size_t out_buf_size) {
    if ((out_buf == NULL) || (len > PROTO_MAX_PAYLOAD) || ((len > 0u) && (payload == NULL))) {
        return 0u;
    }
    size_t total = (size_t)len + 5u;
    if (out_buf_size < total) {
        return 0u;
    }
    out_buf[0] = PROTO_SOF1;
    out_buf[1] = PROTO_SOF2;
    out_buf[2] = cmd;
    out_buf[3] = len;
    if (len > 0u) {
        memcpy(&out_buf[4], payload, len);
    }
    out_buf[4u + (size_t)len] = proto_calc_checksum(mode, cmd, len, payload);
    return total;
}

/* ===================== 3. VOFA+ JustFloat & Live ASCII Tuner ===================== */

size_t vofa_pack_justfloat(const float *channels, uint8_t ch_cnt, uint8_t *out_buf,
                           size_t out_buf_size) {
    if ((channels == NULL) || (out_buf == NULL) || (ch_cnt == 0u)) {
        return 0u;
    }
    size_t payload_bytes = (size_t)ch_cnt * sizeof(float);
    size_t total_bytes = payload_bytes + 4u;
    if (out_buf_size < total_bytes) {
        return 0u;
    }
    memcpy(out_buf, channels, payload_bytes);
    out_buf[payload_bytes + 0u] = 0x00u;
    out_buf[payload_bytes + 1u] = 0x00u;
    out_buf[payload_bytes + 2u] = 0x80u;
    out_buf[payload_bytes + 3u] = 0x7Fu;
    return total_bytes;
}

static char to_upper_ascii(char c) {
    if ((c >= 'a') && (c <= 'z')) {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static bool key_equals_nocase(const char *a, size_t a_len, const char *b) {
    size_t i = 0u;
    while ((i < a_len) && (b[i] != '\0')) {
        if (to_upper_ascii(a[i]) != to_upper_ascii(b[i])) {
            return false;
        }
        i++;
    }
    return (i == a_len) && (b[i] == '\0');
}

/* Lightweight single-precision float parser: zero double math library pull, zero malloc. */
static bool parse_float_f32(const char *s, float *out_val) {
    if ((s == NULL) || (out_val == NULL)) {
        return false;
    }
    while ((*s == ' ') || (*s == '\t')) {
        s++;
    }
    float sign = 1.0f;
    if (*s == '-') {
        sign = -1.0f;
        s++;
    } else if (*s == '+') {
        s++;
    }

    bool has_digit = false;
    float val = 0.0f;
    while ((*s >= '0') && (*s <= '9')) {
        has_digit = true;
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        float frac_scale = 0.1f;
        while ((*s >= '0') && (*s <= '9')) {
            has_digit = true;
            val += (float)(*s - '0') * frac_scale;
            frac_scale *= 0.1f;
            s++;
        }
    }

    if (!has_digit) {
        return false;
    }
    while ((*s == ' ') || (*s == '\t') || (*s == '\r') || (*s == '\n')) {
        s++;
    }
    if (*s != '\0') {
        return false;
    }
    *out_val = sign * val;
    return isfinite(*out_val);
}

int32_t tuner_parse_line(const char *line, const tuner_param_t *table, size_t count) {
    if ((line == NULL) || (table == NULL) || (count == 0u)) {
        return -1;
    }
    while ((*line == ' ') || (*line == '\t')) {
        line++;
    }
    const char *eq = strchr(line, '=');
    if (eq == NULL) {
        eq = strchr(line, ':');
    }
    if ((eq == NULL) || (eq == line)) {
        return -1;
    }

    const char *key_end = eq;
    while ((key_end > line) && ((*(key_end - 1) == ' ') || (*(key_end - 1) == '\t'))) {
        key_end--;
    }
    size_t key_len = (size_t)(key_end - line);
    if (key_len == 0u) {
        return -1;
    }

    float parsed = 0.0f;
    if (!parse_float_f32(eq + 1, &parsed)) {
        return -1;
    }

    for (size_t i = 0u; i < count; i++) {
        if ((table[i].name == NULL) || (table[i].target_ptr == NULL)) {
            continue;
        }
        if (key_equals_nocase(line, key_len, table[i].name)) {
            if (parsed < table[i].min_val) {
                parsed = table[i].min_val;
            } else if (parsed > table[i].max_val) {
                parsed = table[i].max_val;
            }
            *(table[i].target_ptr) = parsed;
            return (int32_t)i;
        }
    }
    return -1;
}
