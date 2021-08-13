#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <serial.h>

void set_motors_hal(int16_t left, int16_t right) {
    char msg[32];
    sprintf(msg, "Motors: %d %d\r\n", left, right);
    serial_send_blocking(msg, strlen(msg));
}