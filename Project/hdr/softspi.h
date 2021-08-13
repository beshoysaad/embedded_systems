#ifndef EMBEDDEDSYSTEMS18_SOFTSPI_H
#define EMBEDDEDSYSTEMS18_SOFTSPI_H

#include <stdint.h>

void softspi_setup_master(void);
void softspi_read(uint8_t* s, uint8_t n);
void softspi_write_uint8(uint8_t x);
void softspi_write(const uint8_t* s, uint8_t n);
void softspi_transfer(const uint8_t *wrt_buffer, uint8_t wrt_len, uint8_t *rd_buffer, uint8_t rd_len);

#endif //EMBEDDEDSYSTEMS18_SOFTSPI_H
