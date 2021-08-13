#include <radio.h>
#include <serial.h>
#include <string.h>
#include <stdio.h>

char rx_buf[128];

void Radio_Init() {
}

void Radio_Configure_Rx(RADIO_PIPE pipe, uint8_t* address, ON_OFF enable) {
    serial_receive(rx_buf, sizeof(rx_buf));
}

void Radio_Configure(RADIO_DATA_RATE dr, RADIO_TX_POWER power) {
}

void Radio_Set_Tx_Addr(uint8_t* address) {
}

uint8_t Radio_Transmit(radiopacket_t* payload, uint8_t len, RADIO_TX_WAIT wait) {
    serial_send_blocking("RF:", 3);
    for (uint8_t i = 0; i < len; i++) {
        char temp[3];
        sprintf(temp, " %02x", payload->payload[i]);
        serial_send_blocking(temp, sizeof(temp));
    }
    serial_send_blocking("\r\n", 2);
    return RADIO_TX_SUCCESS;
}

RADIO_RX_STATUS Radio_Receive(radiopacket_t* buffer, uint8_t *len) {
    *len = serial_get_received_bytes();
    if (rx_buf[*len - 1] == '\r') {
        memcpy(buffer->payload, rx_buf, *len);
        serial_cancel_receive();
        serial_receive(rx_buf, sizeof(rx_buf));
        return RADIO_RX_SUCCESS;
    }
    else {
        return RADIO_RX_FIFO_EMPTY;
    }
}

uint8_t Radio_Success_Rate() {
    return 1;
}

void Radio_Flush() {

}