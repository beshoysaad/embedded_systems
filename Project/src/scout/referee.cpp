#include <stdint.h>
#include <radio.h>
#include <serial.h>
#include <string.h>
#include <addresses.h>
#include <stdio.h>

char uart_rx_buf[128];

void radio_rxhandler(uint8_t pipe_number) {

}

int main(void) {
    serial_set_baud_rate(9600);

    serial_set_mode(SERIAL_AUTOMATIC);

    char msg[] = "\r\n\t- Referee Simulator -\t\r\n";

    serial_send_blocking(msg, strlen(msg));

    serial_receive(uart_rx_buf, sizeof(uart_rx_buf));

    Radio_Init();

    uint8_t own_address[] = REFEREE_ADDRESS;

    uint8_t dist_address[] = COLLECTOR_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_0, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

    radiopacket_t test;

    int theta_in, x_in, y_in;
    unsigned theta, x, y;


    while (1) {
        uint8_t num_rx_bytes = serial_get_received_bytes();
        if (num_rx_bytes > 0) {
            if (uart_rx_buf[num_rx_bytes - 1] == '\r') {
                //serial_send_blocking("try\r\n", 5);
                sscanf(uart_rx_buf, "%d %d %d\r\n", &theta_in, &x_in, &y_in);
                theta = (unsigned)((double)theta_in * 65535.0 / 360.0);
                x = x_in * 10;
                y = y_in * 10;
                test.payload[0] = 0x61;
                test.payload[1] = (theta >> 8) & 0xFF;
                test.payload[2] = theta & 0xFF;
                test.payload[3] = (x >> 8) & 0xFF;
                test.payload[4] = x & 0xFF;
                test.payload[5] = (y >> 8) & 0xFF;
                test.payload[6] = y & 0xFF;
                uint8_t ret_val_tx = Radio_Transmit(&test, 7, RADIO_WAIT_FOR_TX);
                if (ret_val_tx != RADIO_TX_SUCCESS) {
                    serial_send_blocking("Tx Failure\r\n", 12);
                }
                memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
                serial_cancel_receive();
                serial_receive(uart_rx_buf, sizeof(uart_rx_buf));
            }
        }
    }
}