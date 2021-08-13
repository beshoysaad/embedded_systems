#include <stdint.h>
#include <radio.h>
#include <addresses.h>
#include <ref_message.h>
#include <string.h>
#include <ids.h>
#include <motors.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <serial.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <util.h>
#include <outcome.h>

#define DEBUG
//#define PRINT_RX_BUF

#define BACKWARD_SPEED              -32
#define TIMER_THRESHOLD_30_S        2500
#define TIMER_THRESHOLD_500_MS      42
#define X_MAX                       140
#define Y_MAX                       80
#define FORWARD_PWM                 64
#define FORWARD_LIN_VEL             35
#define WHEEL_RADIUS                1.5
#define LENGTH                      8.5
#define FORWARD_ANG_VEL             (FORWARD_LIN_VEL / WHEEL_RADIUS)

typedef enum _state_t {
    INIT, IDLE, GEN_TARGET_POS, ROTATE_TOWARD_TARGET, MOVE_TOWARD_TARGET, OUT_OF_BOUNDS, COLLISION
} state_t;

typedef struct _position_t {
    double x;
    double y;
} position_t;

radiopacket_t rx_pkt, tx_pkt;
uint8_t ref_address[] = REFEREE_ADDRESS;
uint8_t collector_address[] = COLLECTOR_ADDRESS;
uint8_t scout_address[] = SCOUT_ADDRESS;
volatile bool oob_timer_fired = false;
volatile bool coll_timer_fired = false;
volatile uint16_t timer_threshold = 0;
char report[128];
state_t state = INIT;
position_t current_pos, target_pos;
double current_angle, ang_vel_r, ang_vel_l, rot_angle, past_rot_angle, rem_distance, past_rem_distance;
unsigned long current_time, last_time, delta_time;

inline void print_serial(char* buf) {
#ifdef DEBUG
    serial_send_blocking(buf, strlen(buf));
#endif
}

inline double get_distance(position_t p1, position_t p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

inline double get_angle(position_t p1, position_t p2) {
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

inline void start_timer(void) {
    TCCR0A |= (1 << WGM01);                 // CTC mode
    OCR0A = 255;
    TIMSK0 |= (1 << OCIE0A);                // Enable compare interrupt
    sei();                                  // Enable global interrupts
    TCCR0B |= (1 << CS02) | (1 << CS00);    // Start timer. Prescaler 1024
}

inline void stop_timer(void) {
    TCCR0B = 0;
    TCNT0 = 0;
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
        else {
            print_serial("Possible memory corruption detected!\r\n");
            while(1);
        }
        stop_timer();
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
#ifdef PRINT_RX_BUF
        print_serial("Rx: ");
        for (uint8_t i = 0; i < rx_len; i++) {
            sprintf(report, "%02X ", rx_pkt.payload[i]);
            print_serial(report);
        }
        print_serial("\r\n");
#endif
        switch (rx_pkt.payload[0]) {
        case CONFIG: {
            uint8_t channel = rx_pkt.payload[1];
            Radio_Set_Channel(channel);
            break;
        }
        case GO: {
            state = GEN_TARGET_POS;
            break;
        }
        case END: {
            set_motors(0, 0);
            switch (rx_pkt.payload[1]) {
            case OUTCOME_LOSS: {
                print_serial("\r\n\t- Game Lost -\t\r\n");
                break;
            }
            case OUTCOME_WIN: {
                print_serial("\r\n\t- Game Won -\t\r\n");
                break;
            }
            case OUTCOME_TIE: {
                print_serial("\r\n\t- Game Tied -\t\r\n");
                break;
            }
            case OUTCOME_DISQUAL: {
                print_serial("\r\n\t- Disqualified -\t\r\n");
                break;
            }
            }
            state = IDLE;
            break;
        }
        case PING: {
            uint16_t nonce = (((uint16_t) rx_pkt.payload[1]) << 8) | rx_pkt.payload[2];
            nonce++;
            tx_pkt.payload[0] = PONG;
            tx_pkt.payload[1] = (nonce >> 8) & 0xFF;
            tx_pkt.payload[2] = nonce & 0xFF;
            Radio_Transmit(&tx_pkt, 3, RADIO_WAIT_FOR_TX);
            break;
        }
        case POS: {
            uint16_t theta = (((uint16_t) rx_pkt.payload[1]) << 8) | rx_pkt.payload[2];
            int16_t x = (((int16_t) rx_pkt.payload[3]) << 8) | rx_pkt.payload[4];
            int16_t y = (((int16_t) rx_pkt.payload[5]) << 8) | rx_pkt.payload[6];
            current_angle = theta;
            current_angle = (current_angle / 65535) * 360;
            current_angle -= 90.0;
            current_angle *= -1;
            if (current_angle < -180.0) {
                current_angle += 360.0;
            }
            current_angle *= DEG_TO_RAD;
            current_pos.x = x;
            current_pos.y = y;
            current_pos.x /= 10;
            current_pos.y /= 10;
            sprintf(report, "pos %d, %d, %d\r\n", (int) (current_angle * RAD_TO_DEG), (int) current_pos.x,
                    (int) current_pos.y);
            print_serial(report);
            break;
        }
        case OOB: {
            set_motors(0, 0);
            uint16_t theta = (((uint16_t) rx_pkt.payload[1]) << 8) | rx_pkt.payload[2];
            int16_t x = (((int16_t) rx_pkt.payload[3]) << 8) | rx_pkt.payload[4];
            int16_t y = (((int16_t) rx_pkt.payload[5]) << 8) | rx_pkt.payload[6];
            current_angle = theta;
            current_angle = (current_angle / 65535) * 360;
            current_angle -= 90.0;
            current_angle *= -1;
            if (current_angle < -180.0) {
                current_angle += 360.0;
            }
            current_angle *= DEG_TO_RAD;
            current_pos.x = x;
            current_pos.y = y;
            current_pos.x /= 10;
            current_pos.y /= 10;
            sprintf(report, "oob %d, %d, %d\r\n", (int) (current_angle * RAD_TO_DEG), (int) current_pos.x,
                    (int) current_pos.y);
            print_serial(report);
            if (state == COLLISION) {
                stop_timer();
            }
            timer_threshold = TIMER_THRESHOLD_30_S;
            start_timer();
            state = OUT_OF_BOUNDS;
            break;
        }
        case COLL: {
            if (state == OUT_OF_BOUNDS) {
                break;
            }
            set_motors(BACKWARD_SPEED, BACKWARD_SPEED);
            uint16_t theta = (((uint16_t) rx_pkt.payload[1]) << 8) | rx_pkt.payload[2];
            int16_t x = (((int16_t) rx_pkt.payload[3]) << 8) | rx_pkt.payload[4];
            int16_t y = (((int16_t) rx_pkt.payload[5]) << 8) | rx_pkt.payload[6];
            current_angle = theta;
            current_angle = (current_angle / 65535) * 360;
            current_angle -= 90.0;
            current_angle *= -1;
            if (current_angle < -180.0) {
                current_angle += 360.0;
            }
            current_angle *= DEG_TO_RAD;
            current_pos.x = x;
            current_pos.y = y;
            current_pos.x /= 10;
            current_pos.y /= 10;
            sprintf(report, "coll %d, %d, %d\r\n", (int) (current_angle * RAD_TO_DEG), (int) current_pos.x,
                    (int) current_pos.y);
            print_serial(report);
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
}

RADIO_TX_STATUS send_to_referee(radiopacket_t *pkt, uint8_t len) {
    Radio_Set_Tx_Addr(ref_address);
    return Radio_Transmit(pkt, len, RADIO_WAIT_FOR_TX);
}

RADIO_TX_STATUS send_to_collector(radiopacket_t *pkt, uint8_t len) {
    Radio_Set_Tx_Addr(collector_address);
    return Radio_Transmit(pkt, len, RADIO_WAIT_FOR_TX);
}

int main(void) {
    Radio_Init();
    Radio_Configure_Rx(RADIO_PIPE_1, scout_address, ENABLE);
    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);
    serial_set_baud_rate(9600);
    print_serial("\r\n\t- Program Start -\t\r\n");
    while (1) {
        current_time = millis();
        delta_time = current_time - last_time;
        last_time = current_time;
        switch (state) {
        case INIT: {
            tx_pkt.payload[0] = HELLO;
            tx_pkt.payload[1] = SCOUT_ID;
            RADIO_TX_STATUS tx_status = send_to_referee(&tx_pkt, 2);
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
        case GEN_TARGET_POS: {
            target_pos.x = rand() % X_MAX;
            target_pos.y = rand() % Y_MAX;
            sprintf(report, "target %d, %d\r\n", (int) target_pos.x, (int) target_pos.y);
            print_serial(report);
            rot_angle = get_angle(current_pos, target_pos) - current_angle;
            while (rot_angle > PI) {
                rot_angle -= (2 * PI);
            }
            while (rot_angle < -PI) {
                rot_angle += (2 * PI);
            }
            past_rot_angle = rot_angle;
            past_rem_distance = rem_distance = get_distance(current_pos, target_pos);
            if (rot_angle < 0) {
                set_motors(FORWARD_PWM, -FORWARD_PWM);
                ang_vel_l = FORWARD_ANG_VEL;
                ang_vel_r = -FORWARD_ANG_VEL;
            } else {
                set_motors(-FORWARD_PWM, FORWARD_PWM);
                ang_vel_l = -FORWARD_ANG_VEL;
                ang_vel_r = FORWARD_ANG_VEL;
            }
            state = ROTATE_TOWARD_TARGET;
            break;
        }
        case ROTATE_TOWARD_TARGET: {
            if (abs(rot_angle) <= abs(past_rot_angle)) {
                double angle_delta = WHEEL_RADIUS * delta_time * (ang_vel_r - ang_vel_l) / (LENGTH * 1000.0);
                past_rot_angle = rot_angle;
                rot_angle -= angle_delta;
                current_angle += angle_delta;
                sprintf(report, "rot angle %d\r\n", (int) (rot_angle * RAD_TO_DEG));
                print_serial(report);
            } else {
                set_motors(FORWARD_PWM, FORWARD_PWM);
                ang_vel_l = ang_vel_r = FORWARD_ANG_VEL;
                state = MOVE_TOWARD_TARGET;
            }
            break;
        }
        case MOVE_TOWARD_TARGET: {
            if (rem_distance <= past_rem_distance) {
                current_pos.x += WHEEL_RADIUS * delta_time * (ang_vel_l + ang_vel_r) * cos(current_angle) / 2000.0;
                current_pos.y += WHEEL_RADIUS * delta_time * (ang_vel_l + ang_vel_r) * sin(current_angle) / 2000.0;
                past_rem_distance = rem_distance;
                rem_distance = get_distance(current_pos, target_pos);
                sprintf(report, "rem dist %d\r\n", (int) rem_distance);
                print_serial(report);
            } else {
                set_motors(0, 0);
                state = GEN_TARGET_POS;
            }
            break;
        }
        case OUT_OF_BOUNDS: {
            if (oob_timer_fired == true) {
                oob_timer_fired = false;
                state = GEN_TARGET_POS;
            }
            break;
        }
        case COLLISION: {
            if (coll_timer_fired == true) {
                coll_timer_fired = false;
                set_motors(0, 0);
                state = GEN_TARGET_POS;
            }
            break;
        }
        default: {
            break;
        }
        }
    }
}
