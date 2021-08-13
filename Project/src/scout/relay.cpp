#include <stdint.h>
#include <serial.h>
#include <string.h>
#include <radio.h>
#include <addresses.h>
#include <packet.h>
#include <light.h>
#include <stdio.h>
#include <time.h>
#include <avr/interrupt.h>

#define UART_RX_BUF_SZ      16

char uart_rx_buf[UART_RX_BUF_SZ];
volatile bool timer_fired = false;

void radio_rxhandler(uint8_t pipe_number) {
    radiopacket_t rx_pkt;
    RADIO_RX_STATUS ret_val;
    do {
        ret_val = Radio_Receive(&rx_pkt);
        serial_send_blocking("Rx: ", 4);
        serial_send_blocking((char*) rx_pkt.payload.message.messagecontent,
                strlen((char*) rx_pkt.payload.message.messagecontent));
        serial_send_blocking("\r\n", 2);
    } while (ret_val == RADIO_RX_MORE_PACKETS);
}

void init_timer(void) {
    TCCR0A |= (1 << WGM01); // CTC mode
    OCR0A = 255;
    TIMSK0 |= (1 << OCIE0A); // Enable compare interrupt
    sei(); // Enable global interrupts
    TCCR0B |= (1 << CS02) | (1 << CS00); // Start timer. Prescaler 1024
}

ISR(TIMER0_COMPA_vect) {
    static uint8_t count = 0;
    if ((count % 32) == 0) {
        timer_fired = true;
        count = 0;
    }
    count++;
}

int main(void) {
    serial_set_baud_rate(9600);

    serial_set_mode(SERIAL_AUTOMATIC);

    char msg[] = "\r\n\t- Program Start -\t\r\n";

    serial_send_blocking(msg, strlen(msg));

    serial_receive(uart_rx_buf, UART_RX_BUF_SZ);

    Radio_Init();

    uint8_t own_address[] = SCOUT_ADDRESS;

    uint8_t dist_address[] = COLLECTOR_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_1, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

    radiopacket_t test;

    test.type = MESSAGE;
    test.timestamp = 0;
    test.payload.message.messageid = 0x5;

    init_light_sensors();
    init_timer();
    while (1) {
        uint8_t num_rx_bytes = serial_get_received_bytes();
        if (num_rx_bytes > 0) {
            if (uart_rx_buf[num_rx_bytes - 1] == '\r') {
                uart_rx_buf[num_rx_bytes] = '\0';
                serial_send_blocking("Tx: ", 4);
                serial_send_blocking(uart_rx_buf, num_rx_bytes);
                serial_send_blocking("\r\n", 2);
                memcpy(test.payload.message.messagecontent, uart_rx_buf, num_rx_bytes + 1);
                uint8_t ret_val_tx = Radio_Transmit(&test, RADIO_WAIT_FOR_TX);

                if (ret_val_tx == RADIO_TX_SUCCESS) {
                } else {
                    serial_send_blocking("Tx Failure\r\n", 12);
                }

                serial_cancel_receive();
                serial_receive(uart_rx_buf, UART_RX_BUF_SZ);
            }
        }
        if (timer_fired) {
            timer_fired = false;
            uint8_t sensors[NUM_SENSORS];
            read_light_sensors(sensors, NUM_SENSORS);
            char report[64];
            sprintf(report, "front %d, right %d, back %d, left %d\r\n", sensors[FRONT], sensors[RIGHT],
                    sensors[BACK], sensors[LEFT]);
            serial_send_blocking(report, strlen(report));
        }
    }
}
