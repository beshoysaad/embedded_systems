#include <pins.h>
#include "../libs/libpololu/pololu/time.h"
#include <stdint.h>
#include <softspi.h>
#include <stddef.h>

#define SOFTSPI_MODE_0

#ifndef SOFTSPI_DONT_USE_MISO
#define SOFTSPI_DONT_USE_MISO 0
#endif

void softspi_setup_master(void) {
    static bool already_setup = false;
    if (!already_setup) {
        already_setup = true;
        SOFTSPI_CLK_DDR |= SOFTSPI_CLK_MASK;
        SOFTSPI_MOSI_DDR |= SOFTSPI_MOSI_MASK;

#if (SOFTSPI_DONT_USE_MISO == 0)
        SOFTSPI_MISO_DDR &= ~SOFTSPI_MISO_MASK;
#endif
    }
}

static inline void softspi_clk_low(void) {
    SOFTSPI_CLK_PORT &= ~SOFTSPI_CLK_MASK;
    delay_us(1);
}

static inline void softspi_clk_high(void) {
    SOFTSPI_CLK_PORT |= SOFTSPI_CLK_MASK;
    delay_us(1);
}

static inline void softspi_mosi_low(void) {
    SOFTSPI_MOSI_PORT &= ~SOFTSPI_MOSI_MASK;
}

static inline void softspi_mosi_high(void) {
    SOFTSPI_MOSI_PORT |= SOFTSPI_MOSI_MASK;
}

#if (SOFTSPI_DONT_USE_MISO == 0)
static inline uint8_t softspi_is_miso(void) {
    return SOFTSPI_MISO_PIN & SOFTSPI_MISO_MASK;
}
#endif

static inline void softspi_set_sck_freq(uint8_t x) {
}

static inline void softspi_write_bit(uint8_t x, uint8_t m) {
#if defined(SOFTSPI_MODE_0) /* cpol == 0, cpha == 0 */
    if (x & m)
        softspi_mosi_high();
    else
        softspi_mosi_low();
#endif

    softspi_clk_high();

#if !defined(SOFTSPI_MODE_0)
    if (x & m) softspi_mosi_high(); else softspi_mosi_low();
#endif

    softspi_clk_low();
}

static inline void softspi_transfer_bit(uint8_t wrt_byte, uint8_t wrt_msk, uint8_t *rd_byte, uint8_t rd_idx) {
#if defined(SOFTSPI_MODE_0) /* cpol == 0, cpha == 0 */
    if (wrt_byte & wrt_msk)
        softspi_mosi_high();
    else
        softspi_mosi_low();
#endif

    softspi_clk_high();

#if defined(SOFTSPI_MODE_0) /* cpol == 0, cpha == 0 */
    if (softspi_is_miso())
        *rd_byte |= 1 << rd_idx;
#endif

#if !defined(SOFTSPI_MODE_0)
    if (wrt_byte & wrt_msk) softspi_mosi_high(); else softspi_mosi_low();
#endif

    softspi_clk_low();

#if !defined(SOFTSPI_MODE_0)
    if (softspi_is_miso()) *rd_byte |= 1 << rd_idx;
#endif
}

void softspi_write_uint8(uint8_t x) {
    softspi_write_bit(x, (1 << 7));
    softspi_write_bit(x, (1 << 6));
    softspi_write_bit(x, (1 << 5));
    softspi_write_bit(x, (1 << 4));
    softspi_write_bit(x, (1 << 3));
    softspi_write_bit(x, (1 << 2));
    softspi_write_bit(x, (1 << 1));
    softspi_write_bit(x, (1 << 0));
}

static inline void softspi_write_uint16(uint16_t x) {
    softspi_write_uint8((uint8_t)(x >> 8));
    softspi_write_uint8((uint8_t)(x & 0xff));
}

void softspi_write(const uint8_t* s, uint8_t n) {
    if (s == NULL) {
        return;
    }
    for (; n != 0; --n, ++s)
        softspi_write_uint8(*s);
}

void softspi_transfer(const uint8_t *wrt_buffer, uint8_t wrt_len, uint8_t *rd_buffer, uint8_t rd_len) {
    if ((wrt_buffer == NULL) && (rd_buffer == NULL)) {
        return;
    } else if ((wrt_buffer == NULL) && (rd_buffer != NULL)) {
        softspi_read(rd_buffer, rd_len);
    } else if ((rd_buffer == NULL) && (wrt_buffer != NULL)) {
        softspi_write(wrt_buffer, wrt_len);
    } else {
        uint8_t len = wrt_len < rd_len ? wrt_len : rd_len;
        for (; len != 0; --len, ++wrt_buffer, ++rd_buffer) {
            *rd_buffer = 0;
            softspi_transfer_bit(*wrt_buffer, (1 << 7), rd_buffer, 7);
            softspi_transfer_bit(*wrt_buffer, (1 << 6), rd_buffer, 6);
            softspi_transfer_bit(*wrt_buffer, (1 << 5), rd_buffer, 5);
            softspi_transfer_bit(*wrt_buffer, (1 << 4), rd_buffer, 4);
            softspi_transfer_bit(*wrt_buffer, (1 << 3), rd_buffer, 3);
            softspi_transfer_bit(*wrt_buffer, (1 << 2), rd_buffer, 2);
            softspi_transfer_bit(*wrt_buffer, (1 << 1), rd_buffer, 1);
            softspi_transfer_bit(*wrt_buffer, (1 << 0), rd_buffer, 0);
        }
        if (rd_len < wrt_len) {
            softspi_write(wrt_buffer, wrt_len - rd_len);
        } else if (wrt_len < rd_len) {
            softspi_read(rd_buffer, rd_len - wrt_len);
        }
    }
}

#if (SOFTSPI_DONT_USE_MISO == 0)

static inline void softspi_read_bit(uint8_t* x, uint8_t i) {
    softspi_clk_high();

#if defined(SOFTSPI_MODE_0) /* cpol == 0, cpha == 0 */
    if (softspi_is_miso())
        *x |= 1 << i;
#endif

    softspi_clk_low();

#if !defined(SOFTSPI_MODE_0)
    if (softspi_is_miso()) *x |= 1 << i;
#endif
}

static uint8_t softspi_read_uint8(void) {
    /* must be initialized to 0 */
    uint8_t x = 0;

    softspi_read_bit(&x, 7);
    softspi_read_bit(&x, 6);
    softspi_read_bit(&x, 5);
    softspi_read_bit(&x, 4);
    softspi_read_bit(&x, 3);
    softspi_read_bit(&x, 2);
    softspi_read_bit(&x, 1);
    softspi_read_bit(&x, 0);

    return x;
}

static inline uint16_t softspi_read_uint16(void) {
    const uint8_t x = softspi_read_uint8();
    return ((uint16_t) x << 8) | (uint16_t) softspi_read_uint8();
}

void softspi_read(uint8_t* s, uint8_t n) {
    if (s == NULL) {
        return;
    }
    for (; n != 0; --n, ++s)
        *s = softspi_read_uint8();
}

#endif /* SOFTSPI_DONT_USE_MISO == 0 */
