#include <stdint.h>
#include <Arduino.h>
#include <radio.h>
#include <addresses.h>

radiopacket_t test;
volatile bool msg_rx = false;

void radio_rxhandler(uint8_t pipe_number) {
    radiopacket_t rx_pkt;
    uint8_t len;
    Radio_Receive(&rx_pkt, &len);
    memcpy(test.payload, rx_pkt.payload, len);
    msg_rx = true;
}

void setup() {
    Serial1.begin(9600);
    Serial1.print("\r\n\t- Program Start -\t\r\n");
    Radio_Init();

    uint8_t own_address[] = COLLECTOR_ADDRESS;

    uint8_t dist_address[] = SCOUT_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_1, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

}

void loop() {
    if (msg_rx) {
        msg_rx = false;

        if (test.payload[2] == 0x50) {
            test.payload[2]++;
            uint16_t temp = test.payload[0] | ((uint16_t)test.payload[1] << 8);
            temp++;
            test.payload[0] = temp & 0xFF;
            test.payload[1] = (temp >> 8) & 0xFF;

            uint8_t ret_val_tx = Radio_Transmit(&test, 3, RADIO_WAIT_FOR_TX);

            if (ret_val_tx == RADIO_TX_SUCCESS) {
                Serial1.print("RADIO_TX_SUCCESS\r\n");
            } else if (ret_val_tx == RADIO_TX_MAX_RT) {
                Serial1.print("RADIO_TX_MAX_RT\r\n");
            } else {
                Serial1.print("RADIO_TX_UNKNOWN\r\n");
            }
        }
    }
}
