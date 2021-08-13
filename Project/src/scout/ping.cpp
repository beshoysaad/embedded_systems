#include <stdint.h>
#include <serial.h>
#include <string.h>
#include <radio.h>
#include <addresses.h>
#include <packet.h>
#include <light.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define UART_RX_BUF_SZ      16

char uart_rx_buf[UART_RX_BUF_SZ];

void radio_rxhandler(uint8_t pipe_number) {
    radiopacket_t rx_pkt;
    RADIO_RX_STATUS ret_val;
    do {
        uint8_t len;
        ret_val = Radio_Receive(&rx_pkt, &len);
        serial_send_blocking("Rx: ", 4);
        for (int i = 0; i < len; i++) {
            char buf[8];
            sprintf(buf, "0x%02X ", rx_pkt.payload[i]);
            serial_send_blocking(buf, strlen(buf));
        }
        serial_send_blocking("\r\n", 2);
    } while (ret_val == RADIO_RX_MORE_PACKETS);
}

int main(void) {

    serial_set_baud_rate(9600);

    serial_set_mode(SERIAL_AUTOMATIC);

    char msg[] = "\r\n\t- Program Start -\t\r\n";

    serial_send_blocking(msg, strlen(msg));

    serial_receive(uart_rx_buf, UART_RX_BUF_SZ);

    Radio_Init();

    uint8_t own_address[] = SCOUT_ADDRESS;

    uint8_t dist_address[] = REFEREE_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_0, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

    radiopacket_t test;

    while (1) {
        uint8_t num_rx_bytes = serial_get_received_bytes();
        if (num_rx_bytes > 0) {
            if (uart_rx_buf[num_rx_bytes - 1] == '\r') {
                uart_rx_buf[num_rx_bytes] = '\0';
                serial_send_blocking("Tx: ", 4);
                serial_send_blocking(uart_rx_buf, num_rx_bytes);
                serial_send_blocking("\r\n", 2);
                long temp = strtol(uart_rx_buf, NULL, 16);
                memcpy(test.payload, &temp, 3);
                uint8_t ret_val_tx = Radio_Transmit(&test, 3, RADIO_WAIT_FOR_TX);

                if (ret_val_tx == RADIO_TX_SUCCESS) {
                } else {
                    serial_send_blocking("Tx Failure\r\n", 12);
                }

                serial_cancel_receive();
                serial_receive(uart_rx_buf, UART_RX_BUF_SZ);
            }
        }
    }
}
