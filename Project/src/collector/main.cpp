#include <stdint.h>
#include <radio.h>
#include <addresses.h>
#include <ref_message.h>
#include <string.h>
#include <ids.h>
#include <Zumo32U4Motors.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <Arduino.h>

#define DEBUG

#define BACKWARD_SPEED              -32
#define TIMER_THRESHOLD_30_S        2500
#define TIMER_THRESHOLD_500_MS      42

typedef enum _state_t {
    INIT, IDLE, OUT_OF_BOUNDS, COLLISION
} state_t;

radiopacket_t rx_pkt;
uint8_t ref_address[] = REFEREE_ADDRESS;
uint8_t collector_address[] = COLLECTOR_ADDRESS;
uint8_t scout_address[] = SCOUT_ADDRESS;
volatile bool oob_timer_fired = false;
volatile bool coll_timer_fired = false;
volatile bool rx_rdy = false;
uint16_t timer_threshold = 0;
char report[128];
state_t state = INIT;
Zumo32U4Motors motors;

inline void print_serial(char* buf) {
#ifdef DEBUG
    Serial1.print(buf);
#endif
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
    if ((++count % timer_threshold) == 0) {
        if (timer_threshold == TIMER_THRESHOLD_30_S) {
            oob_timer_fired = true;
        }
        else if (timer_threshold == TIMER_THRESHOLD_500_MS) {
            coll_timer_fired = true;
        }
        TCCR0B = 0;
        TCNT0 = 0;
        count = 0;
    }
}

void radio_rxhandler(uint8_t pipenumber) {
    uint8_t rx_len = 0;
    RADIO_RX_STATUS rx_status;
    do {
        rx_status = Radio_Receive(&rx_pkt, &rx_len);
    } while (rx_status == RADIO_RX_MORE_PACKETS);
    if (rx_status == RADIO_RX_SUCCESS) {
#ifdef DEBUG
        print_serial("Rx: ");
        for (uint8_t i = 0; i < rx_len; i++) {
            sprintf(report, "%02X ", rx_pkt.payload[i]);
            print_serial(report);
        }
        print_serial("\r\n");
#endif
        rx_rdy = true;
    }
}

RADIO_TX_STATUS send_to_referee(radiopacket_t *pkt, uint8_t len) {
    Radio_Set_Tx_Addr(ref_address);
    return Radio_Transmit(pkt, len, RADIO_WAIT_FOR_TX);
}

RADIO_TX_STATUS send_to_scout(radiopacket_t *pkt, uint8_t len) {
    Radio_Set_Tx_Addr(scout_address);
    return Radio_Transmit(pkt, len, RADIO_WAIT_FOR_TX);
}

void setup(void) {
    Radio_Init();
    Radio_Configure_Rx(RADIO_PIPE_1, collector_address, ENABLE);
    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);
    Serial1.begin(9600);
    print_serial("\r\n\t- Program Start -\t\r\n");
}

void loop(void) {
    if (rx_rdy == true) {
        rx_rdy = false;
        switch (rx_pkt.payload[0]) {
        case CONFIG: {
            uint8_t channel = rx_pkt.payload[1];
            Radio_Set_Channel(channel);
            break;
        }
        case PING: {
            uint16_t nonce = (((uint16_t) rx_pkt.payload[1]) << 8) | rx_pkt.payload[2];
            nonce++;
            radiopacket_t pong_pkt;
            pong_pkt.payload[0] = PONG;
            pong_pkt.payload[1] = (nonce >> 8) & 0xFF;
            pong_pkt.payload[2] = nonce & 0xFF;
            Radio_Transmit(&pong_pkt, 3, RADIO_WAIT_FOR_TX);
            break;
        }
        case OOB: {
            motors.setSpeeds(0, 0);
            timer_threshold = TIMER_THRESHOLD_30_S;
            start_timer();
            state = OUT_OF_BOUNDS;
            break;
        }
        case COLL: {
            motors.setSpeeds(BACKWARD_SPEED, BACKWARD_SPEED);
            timer_threshold = TIMER_THRESHOLD_500_MS;
            start_timer();
            state = COLLISION;
            break;
        }
        default: {
            break;
        }
        }
    }
    switch (state) {
    case INIT: {
        radiopacket_t hello_pkt;
        hello_pkt.payload[0] = HELLO;
        hello_pkt.payload[1] = COLLECTOR_ID;
        RADIO_TX_STATUS tx_status = send_to_referee(&hello_pkt, 2);
        if (tx_status == RADIO_TX_SUCCESS) {
            state = IDLE;
        } else {
            print_serial("Error sending HELLO message!");
        }
        break;
    }
    case IDLE: {
        break;
    }
    case OUT_OF_BOUNDS: {
        if (oob_timer_fired == true) {
            oob_timer_fired = false;
            state = IDLE;
        }
        break;
    }
    case COLLISION: {
        if (coll_timer_fired == true) {
            coll_timer_fired = false;
            motors.setSpeeds(0, 0);
            state = IDLE;
        }
        break;
    }
    default: {
        break;
    }
    }
}
