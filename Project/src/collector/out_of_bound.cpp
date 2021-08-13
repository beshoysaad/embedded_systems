//
// Created by hp-pc on 06-07-2018.
//

#include <Arduino.h>
#include <Zumo32U4Encoders.h>
#include <util.h>
#include<radio.h>
#include <addresses.h>
#define WHEEL_RADIUS                  2
#define LEFT_MOTOR                    10
#define RIGHT_MOTOR                   9
#define LEFT_MOTOR_DIR                16
#define RIGHT_MOTOR_DIR               15
#define MOTOR_DIR_FORWARD             LOW
#define MOTOR_DIR_REVERSE             HIGH
#define DISTANCE_CORRECTION_FACTOR    10
#define RIGHT_SPEED                   72
#define LEFT_SPEED                    70
#define HOME_X                        0.0
#define HOME_Y                        0.0
#define TURN_ANGLE                    10.0
#define WAIT_TIME                     30000

double angle;
double current_x, current_y;
double current_theta, target_theta, rotation_theta;
unsigned long current_time_ms, last_time_ms,  wait_start_time;;
Zumo32U4Encoders encoders;
typedef enum {
    IDLE,MOVE_OUT_OF_BOUND, WAIT, TURNING, MOVING, END
} state_t;

state_t state = IDLE;



int left_values[] = {
        0,
        111,
        240,
        373,
        499,
        598,
        713,
        801,
        922,
        1005,
        1085,
        1217,
        1259,
        1368,
        1485,
        1550,
        1614,
        1736,
        1806,
        1898,
        1950,
        2015,
        2141,
        2244,
        2323 };

int right_values[] = {
        0,
        120,
        245,
        385,
        512,
        621,
        760,
        885,
        1000,
        1100,
        1200,
        1230,
        1385,
        1471,
        1578,
        1690,
        1800,
        1890,
        1973,
        2144,
        2170,
        2277,
        2436,
        2549,
        2766 };

void turn(double angle) {
    bool negative_angle = false;
    encoders.getCountsAndResetRight();
    encoders.getCountsAndResetLeft();

    if (angle < 0.0) {
        angle = -angle;
        negative_angle = true;
        digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
        digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_REVERSE);
    } else {
        negative_angle = false;
        digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_REVERSE);
        digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    }

    uint8_t angle_index = 0;
    bool angle_found = false;

    while (!angle_found) {
        if ((15.0 * angle_index <= angle) && (angle <= (15.0 * (angle_index + 1)))) {
            angle_found = true;
        }
        angle_index++;
    }

    int16_t left_encoder_value = left_values[angle_index]
                                 + ((left_values[angle_index + 1] - left_values[angle_index]) / 15) * (angle - 15 * angle_index);
    int16_t right_encoder_value = right_values[angle_index]
                                  + ((right_values[angle_index + 1] - right_values[angle_index]) / 15) * (angle - 15 * angle_index);

    Serial1.print("left val:");
    Serial1.print(left_encoder_value);
    Serial1.print(" right val:");
    Serial1.println(right_encoder_value);

    if (!negative_angle) {
        while ((encoders.getCountsLeft() > -left_encoder_value) && (encoders.getCountsRight() < right_encoder_value)) {
            analogWrite(RIGHT_MOTOR, RIGHT_SPEED);
            analogWrite(LEFT_MOTOR, LEFT_SPEED);
        }

    } else {
        while ((encoders.getCountsRight() > (-right_encoder_value)) && (encoders.getCountsLeft() < left_encoder_value)) {
            //while ((encoders.getCountsRight() < (-right_encoder_value)) && (encoders.getCountsLeft() > left_encoder_value)) {
            analogWrite(RIGHT_MOTOR, RIGHT_SPEED);
            analogWrite(LEFT_MOTOR, LEFT_SPEED);
            Serial1.print(encoders.getCountsRight());
            Serial1.println(encoders.getCountsLeft());
        }

    }
    analogWrite(RIGHT_MOTOR, 0);
    analogWrite(LEFT_MOTOR, 0);
    delay(100);
    encoders.getCountsAndResetRight();
    encoders.getCountsAndResetLeft();
}

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
        digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
        digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_FORWARD);
        state= MOVE_OUT_OF_BOUND;

    }
    if (rx_pkt.payload[0] == 0x61) {
        analogWrite(RIGHT_MOTOR, 0);
        analogWrite(LEFT_MOTOR, 0);

        uint16_t theta = (rx_pkt.payload[1] << 8) | rx_pkt.payload[2];
        int16_t x = (rx_pkt.payload[3] << 8) | rx_pkt.payload[4];
        int16_t y = (rx_pkt.payload[5] << 8) | rx_pkt.payload[6];
        current_x = (double) x / 10.0;
        current_y = (double) y / 10.0;
        current_theta = ((double) theta / 65535.0) * 360.0;

        wait_start_time = millis();

        state = WAIT;
    }
}




void setup() {
    Serial1.begin(9600);


    Serial1.print("\r\n\t- Program Start -\t\r\n");

    encoders.init();

    Radio_Init();

    uint8_t own_address[] = COLLECTOR_ADDRESS;

    uint8_t dist_address[] = REFEREE_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_0, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);

   // attachInterrupt(2, RF_rx_ISR, FALLING);
}

void loop() {
    current_time_ms = millis();
    unsigned long delta_ms = current_time_ms - last_time_ms;
    if (delta_ms != 0) {
        last_time_ms = current_time_ms;

        switch (state) {
            case IDLE:
                analogWrite(RIGHT_MOTOR, 0);
                analogWrite(LEFT_MOTOR, 0);
                break;
            case TURNING:
                delay(100);
                Serial1.print("C_Theta : ");
                Serial1.print(current_theta);
                Serial1.print("current_x : ");
                Serial1.print(current_x);
                Serial1.print("current_y : ");
                Serial1.print(current_y);
                Serial1.print("r_theta : ");
                Serial1.print(rotation_theta);
                delay(1000);
                turn(rotation_theta);
                delay(100);
                digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
                digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_FORWARD);
                state = MOVING;
                break;

            case MOVE_OUT_OF_BOUND:
            {
                analogWrite(RIGHT_MOTOR, RIGHT_SPEED / 2);
                analogWrite(LEFT_MOTOR, LEFT_SPEED / 2);

            }
            break;

            case WAIT: {
                if ((current_time_ms - wait_start_time) < WAIT_TIME) {
                    /*do nothing*/
                } else {
                    Serial1.print("Cur_Theta : ");
                    Serial1.print(current_theta);

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
                    state = TURNING;
                }
            }


                break;
            case MOVING: {
                float rem_distance = sqrt(pow((HOME_X - current_x), 2) + pow((HOME_Y - current_y), 2));
                if (rem_distance >= 6) {
                    analogWrite(RIGHT_MOTOR, RIGHT_SPEED);
                    analogWrite(LEFT_MOTOR, LEFT_SPEED);
                } else {
                    analogWrite(RIGHT_MOTOR, 0);
                    analogWrite(LEFT_MOTOR, 0);
                    state = END;
                }

                delay(100);

                int16_t left_encoder = encoders.getCountsAndResetLeft();
                int16_t right_encoder = encoders.getCountsAndResetRight();
                Serial1.print(" Left =");
                Serial1.print(left_encoder);
                Serial1.print(" Right =");
                Serial1.print(right_encoder);
                double angular_l =
                        ((double) left_encoder) * 1000.0 * 60.0 * 0.10472 / (delta_ms * 909.7);
                double angular_r =
                        ((double) right_encoder) * 1000.0 * 60.0 * 0.10472 / (delta_ms * 909.7);
                current_x += WHEEL_RADIUS * (angular_r + angular_l) * cos(DEG_TO_RAD * target_theta)
                             / (2.0 * DISTANCE_CORRECTION_FACTOR);
                current_y += WHEEL_RADIUS * (angular_r + angular_l) * sin(DEG_TO_RAD * target_theta)
                             / (2.0 * DISTANCE_CORRECTION_FACTOR);
                Serial1.print("  x = ");
                Serial1.print(current_x);
                Serial1.print("  y = ");
                Serial1.print(current_y);
                Serial1.print("  Remaining distance = ");
                Serial1.println(rem_distance);

            }
                break;
            case END:
                turn(TURN_ANGLE);
                delay(50);
                turn(-TURN_ANGLE);
                delay(50);
                break;
        }
    }
}
