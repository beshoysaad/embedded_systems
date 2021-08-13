#include <qtr.h>
#include <digital.h>
#include <time.h>
#include <serial.h>
#include <stdio.h>
#include <string.h>
#include <motors.h>

unsigned char pins[] = {IO_C0, IO_C1, IO_C2, IO_C3, IO_C4};
unsigned int sensors[5];
char report[128];
char uart_rx_buf[32];
unsigned int num_lines = 0;
unsigned int num_lines_detected = 0;
bool line_detected = false;

int main(void) {
    serial_set_baud_rate(9600);
    serial_send_blocking("Waiting for input...\r\n", 22);
    qtr_rc_init(pins, 5, 7500, IO_C5);
    serial_receive(uart_rx_buf, 32);

    while(1) {
        unsigned char rx_len = serial_get_received_bytes();
        if (rx_len > 0) {
            if (uart_rx_buf[rx_len - 1] == '\r') {
                sscanf(uart_rx_buf, "%d", &num_lines);
                sprintf(report, "Num lines = %d\r\n", num_lines);
                serial_send_blocking(report, strlen(report));
                set_motors(32, 32);
                while (1) {
                    qtr_read(sensors, QTR_EMITTERS_ON);
                    if ((sensors[2] > 1500) && (line_detected == false)) {
                        line_detected = true;
                        serial_send_blocking("Line begin\r\n", 12);
                    } else if ((sensors[2] < 1000) && (line_detected == true)) {
                        line_detected = false;
                        serial_send_blocking("Line end\r\n", 10);
                        num_lines_detected++;
                        if (num_lines_detected >= num_lines) {
                            sprintf(report, "Passed %d lines\r\n", num_lines_detected);
                            serial_send_blocking(report, strlen(report));
                            set_motors(0, 0);
                            while(1);
                        }
                    }
                }
            }
        }
    }
}