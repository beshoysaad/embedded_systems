#include <radio.h>
#include <addresses.h>
#include <motors_hal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <serial.h>
#include <string.h>
#include <stdio.h>

#define PING        0x50
#define PONG        0x51
#define POS         0x60
#define OOB         0x61

radiopacket_t rx_pkt;
radiopacket_t tx_pkt;
volatile bool rx_rdy = false;
volatile bool timer_fired = false;

void stop_timer(void) {
    TCCR0B = 0;
    TCNT0 = 0;
}

void start_timer(void) {
    TCCR0A |= (1 << WGM01); // CTC mode
    OCR0A = 255;
    TIMSK0 |= (1 << OCIE0A); // Enable compare interrupt
    sei(); // Enable global interrupts
    TCCR0B |= (1 << CS02) | (1 << CS00); // Start timer. Prescaler 1024
}

ISR(TIMER0_COMPA_vect) {
        static uint16_t count = 0;
        if ((++count % 2500) == 0) {
            timer_fired = true;
            count = 0;
        }
}

void radio_rxhandler(uint8_t pipe_number) {
    uint8_t len;
    RADIO_RX_STATUS status = Radio_Receive(&rx_pkt, &len);
    if (status == RADIO_RX_SUCCESS) {
        rx_rdy = true;
    }
}

int main(void) {
    int16_t left_speed = 64;
    int16_t right_speed = 64;
    serial_set_baud_rate(9600);

    serial_set_mode(SERIAL_AUTOMATIC);

    char msg[] = "\r\n\t- Program Start -\t\r\n";

    serial_send_blocking(msg, strlen(msg));

    Radio_Init();

    uint8_t own_address[] = SCOUT_ADDRESS;

    uint8_t dist_address[] = REFEREE_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_0, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

    set_motors_hal(left_speed, right_speed);

    while(1) {
        if (serial_get_received_bytes() > 0) {
            radio_rxhandler(0);
        }
        if (rx_rdy) {
            rx_rdy = false;
            switch(rx_pkt.payload[0]) {
                case PING:
                {
                    tx_pkt.payload[0] = PONG;
                    uint16_t nonce = (((uint16_t)rx_pkt.payload[1]) << 8) | rx_pkt.payload[2];
                    nonce++;
                    tx_pkt.payload[1] = (nonce >> 8) & 0xFF;
                    tx_pkt.payload[2] = nonce & 0xFF;
                    uint8_t retval = Radio_Transmit(&tx_pkt, 3, RADIO_WAIT_FOR_TX);
                    if (retval != RADIO_TX_SUCCESS) {
                        serial_send_blocking("Tx Failure\r\n", 12);
                    }
                }
                    break;
                case POS:
                    break;
                case OOB:
                    set_motors_hal(0, 0);
                    start_timer();
                    break;
            }
        }
        if (timer_fired) {
            timer_fired = false;
            stop_timer();
            set_motors_hal(left_speed, right_speed);
        }
    }
}