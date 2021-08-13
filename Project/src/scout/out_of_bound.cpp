//
// Created by hp-pc on 03-07-2018.
//

#include <3pi.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <util.h>
#include <radio.h>
#include <addresses.h>

#define HOME_X              0.0
#define HOME_Y              0.0
#define LEFT_PWM            60
#define RIGHT_PWM           61
#define WHEEL_RADIUS        0.8
#define LENGTH              8.2
#define LINEAR_VEL          23.0
#define ANGULAR_VAL         (LINEAR_VEL/WHEEL_RADIUS)
#define THRESHOLD           5.0
#define WAIT_TIME           30000

double current_x;
double current_y;
double current_theta;
double target_theta;
double rotation_theta;
double ang_speed_l, ang_speed_r;
int pwm_r, pwm_l;
double rem_distance;
char buffer[64];
unsigned long wait_start_time;

typedef enum {
    IDLE, MOVE_OUT_OF_BOUND, WAIT, TURNING, MOVING, END
} state_t;

state_t state = IDLE;

void radio_rxhandler(uint8_t pipe_number) {
    radiopacket_t rx_pkt;
    RADIO_RX_STATUS ret_val;
    uint8_t len;
    ret_val = Radio_Receive(&rx_pkt, &len);
    if (rx_pkt.payload[0] == 0x60) {
        uint16_t theta = (rx_pkt.payload[1] << 8) | rx_pkt.payload[2];
        uint16_t x = (rx_pkt.payload[3] << 8) | rx_pkt.payload[4];
        uint16_t y = (rx_pkt.payload[5] << 8) | rx_pkt.payload[6];

        current_x = (double) x / 10.0;
        current_y = (double) y / 10.0;
        current_theta = ((double) theta / 65535.0) * 360.0;

        sprintf(buffer, "current_theta = %d x = %d y = %d\r\n ", (int) current_theta, (int)current_x , (int) current_y);
        serial_send_blocking(buffer, strlen(buffer));
        pwm_l = LEFT_PWM / 2;
        pwm_r = RIGHT_PWM / 2;
        ang_speed_l = ang_speed_r = ANGULAR_VAL / 2;
        state = MOVE_OUT_OF_BOUND;
    }

    if (rx_pkt.payload[0] == 0x61) {
        pwm_l = pwm_r = 0;
        set_motors(pwm_l,pwm_r);

        uint16_t theta = (rx_pkt.payload[1] << 8) | rx_pkt.payload[2];
        int16_t x = (rx_pkt.payload[3] << 8) | rx_pkt.payload[4];
        int16_t y = (rx_pkt.payload[5] << 8) | rx_pkt.payload[6];



        current_x = (double) x / 10.0;
        current_y = (double) y / 10.0;
        current_theta = ((double) theta / 65535.0) * 360.0;


        sprintf(buffer, "current_theta = %d x = %d y = %d\r\n ", (int) current_theta, (int)current_x , (int) current_y);
        serial_send_blocking(buffer, strlen(buffer));

          wait_start_time=millis();
          state = WAIT;
    }
}

int main(void) {

    unsigned long current_time, last_time, delta_time ;
    current_time = last_time = 0;

    serial_set_baud_rate(9600);
    char start_msg[] = "\r\n\t- Program Start -\t\r\n";
    serial_send_blocking(start_msg, strlen(start_msg));

    Radio_Init();

    uint8_t own_address[] = SCOUT_ADDRESS;

    uint8_t dist_address[] = REFEREE_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_0, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

    while (1) {
        delta_time = current_time - last_time;
        last_time = current_time;
        switch (state) {
            case END:
                while (1) {
                    pwm_l = -LEFT_PWM;
                    pwm_r = RIGHT_PWM;
                    set_motors(pwm_l, pwm_r);
                    delay(100);
                    pwm_l = LEFT_PWM;
                    pwm_r = -RIGHT_PWM;
                    set_motors(pwm_l, pwm_r);
                    delay(100);
                }
            case IDLE:
              /*  pwm_l = LEFT_PWM ;
                pwm_r = RIGHT_PWM ;
                set_motors(pwm_l, pwm_r);*/
                serial_send_blocking("IDLE",5);
                break;
            case TURNING:
              //  serial_send_blocking("TURNING",8);
                current_theta += WHEEL_RADIUS * delta_time * (ang_speed_r - ang_speed_l) / (LENGTH * 1000.0);

                rotation_theta = (target_theta - current_theta);

                sprintf(buffer, "theta = %d r_theta = %d\r\n ", (int) (RAD_TO_DEG * current_theta),
                        (int) (RAD_TO_DEG * rotation_theta));
                if (rotation_theta * RAD_TO_DEG < -180) {
                    rotation_theta += 2 * PI;
                }

                if (fabs(rotation_theta * RAD_TO_DEG) < THRESHOLD) {
                    ang_speed_l = ang_speed_r = pwm_l = pwm_r = 0;
                    state = MOVING;
                    delay(20);
                }

                set_motors(pwm_l, pwm_r);
                break;

            case MOVING: {
           //     serial_send_blocking("MOVING",7);
                float previous_dist = rem_distance;
                current_x += WHEEL_RADIUS * delta_time * (ang_speed_l + ang_speed_r) * cos(current_theta) / 2000.0;
                current_y += WHEEL_RADIUS * delta_time * (ang_speed_l + ang_speed_r) * sin(current_theta) / 2000.0;
                sprintf(buffer, "x = %d y = %d\r\n", (int) current_x, (int) current_y);
//            serial_send_blocking(buffer, strlen(buffer));

                rem_distance = sqrt(pow((HOME_X - current_x), 2) + pow((HOME_Y - current_y), 2));
                sprintf(buffer, "rem dist = %d\r\n", (int) rem_distance); //, (int)current_y);
                serial_send_blocking(buffer, strlen(buffer));

                if (rem_distance < THRESHOLD) {
                    ang_speed_l = ang_speed_r = 0;
                    pwm_l = pwm_r = 0;
                    state = END;
                    delay(200);
                } else if (rem_distance > previous_dist) {
                    state = TURNING;
                    pwm_r = pwm_l = 0;
                    target_theta = atan2((HOME_Y - current_y), (HOME_X - current_x));
                    delay(200);
                } else {
                    pwm_l = LEFT_PWM;
                    pwm_r = RIGHT_PWM;
                    ang_speed_l = ang_speed_r = ANGULAR_VAL;
                }
                set_motors(pwm_l, pwm_r);
            }
                break;

                case MOVE_OUT_OF_BOUND:
                    {
                      //  serial_send_blocking("MoveOOB",8);
                    current_x += WHEEL_RADIUS * delta_time * (ang_speed_l + ang_speed_r) * cos(current_theta) / 2000.0;
                    current_y += WHEEL_RADIUS * delta_time * (ang_speed_l + ang_speed_r) * sin(current_theta) / 2000.0;
                    set_motors(pwm_l, pwm_r);
                }
                break;

            case WAIT:
            //    serial_send_blocking("wait",5);
                if((current_time - wait_start_time) < WAIT_TIME){

                    /*wait*/
                }
                else{
                    current_theta -= 90.0;
                    current_theta *= -1;
                    if (current_theta < -180.0) {
                        current_theta += 360.0;
                    }

                    target_theta = RAD_TO_DEG * atan2((HOME_Y - current_y), (HOME_X - current_x));
                    rotation_theta = (target_theta - current_theta);
                    if (rotation_theta < -180.0) {
                        rotation_theta += 360.0;
                    }

                    char msg[64];
                    sprintf(msg, "theta = %d x = %d y= %d r_theta=%d\r\n ", (int) current_theta, (int) current_x, (int) current_y,
                            (int) rotation_theta);
                    serial_send_blocking(msg, strlen(msg));

                    if (rotation_theta < 0) {
                        ang_speed_l = ANGULAR_VAL;
                        ang_speed_r = -ANGULAR_VAL;
                        pwm_l = LEFT_PWM;
                        pwm_r = -RIGHT_PWM;
                    } else {
                        ang_speed_l = -ANGULAR_VAL;
                        ang_speed_r = ANGULAR_VAL;
                        pwm_l = -LEFT_PWM;
                        pwm_r = RIGHT_PWM;
                    }

                    current_theta *= DEG_TO_RAD;
                    target_theta *= DEG_TO_RAD;
                    rotation_theta *= DEG_TO_RAD;
                    rem_distance = sqrt(pow((HOME_X - current_x), 2) + pow((HOME_Y - current_y), 2));
                    state = TURNING;
                }
                break;

        }
        delay(10);
        current_time = millis();
    }
}
