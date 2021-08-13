#include <Arduino.h>

#define UART_RX_BUF_SIZE              64
#define MAX_WAYPOINTS                 8
#define LEFT_MOTOR_DIR                16
#define RIGHT_MOTOR_DIR               15
#define MOTOR_DIR_FORWARD             LOW
#define MOTOR_DIR_REVERSE             HIGH
#define DEGREE_PER_SECOND             360
#define CM_PER_SECOND                 65

typedef struct {
    uint16_t x;
    uint16_t y;
} position_t;

uint8_t waypoint_rd_idx = 0;
uint8_t waypoint_wr_idx = 1;
char uart_rx_buf[UART_RX_BUF_SIZE];
position_t waypoints[MAX_WAYPOINTS], current_position;
int16_t current_orientation;

void parse_waypoint_msg(char* buf, size_t rd_sz) {
    uint8_t rd_idx = 0;
    while (rd_idx < rd_sz) {
        sscanf(&buf[rd_idx], "%u, %u", &waypoints[waypoint_wr_idx].x, &waypoints[waypoint_wr_idx].y);
        char temp[64];
        sprintf(temp, "x: %u, y: %u\r\n", waypoints[waypoint_wr_idx].x, waypoints[waypoint_wr_idx].y);
        Serial1.print(temp);
        if ((waypoint_wr_idx + 1) % MAX_WAYPOINTS != waypoint_rd_idx) {
            waypoint_wr_idx = (waypoint_wr_idx + 1) % MAX_WAYPOINTS;
        } else {
            break;
        }
        while ((buf[rd_idx] != ';') && (rd_idx < rd_sz)) {
            rd_idx++;
        }
        rd_idx++;
    }
}

void turn_towards_target(int16_t angle, int increment_cur_angle) {
    if (angle == 0) {
        return;
    }
    unsigned long angle_abs = abs(angle);
    unsigned long time = (angle_abs * 1000) / DEGREE_PER_SECOND;
    if (angle > 0) {
        digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_REVERSE);
        digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    } else {
        digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
        digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_REVERSE);
    }
    PORTB |= 0b01100000;
    delay(time);
    PORTB &= 0b10011111;
    if (increment_cur_angle) {
        current_orientation += angle;
        while (current_orientation > 360) {
            current_orientation -= 360;
        }
        while (current_orientation < -360) {
            current_orientation += 360;
        }
    }
}

void move_to_target(uint16_t distance) {
    if (distance == 0) {
        return;
    }
    unsigned long time = (distance * 1000) / CM_PER_SECOND;
    digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    PORTB |= 0b01100000;
    delay(time);
    PORTB &= 0b10011111;
    current_position.x += (uint16_t)((double) distance * cos(((double) current_orientation * 3.14159) / 180.0));
    current_position.y += (uint16_t)((double) distance * sin(((double) current_orientation * 3.14159) / 180.0));
}

void rotate_indefinitely(void) {
    digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_REVERSE);
    PORTB |= 0b01100000;
    while (1)
        ;
}

void setup() {
    Serial1.begin(9600);
    Serial1.setTimeout(20000);
    Serial1.print("\r\n\t- Program Start -\t\r\n");
}

void loop() {
    if (Serial1.available()) {
        size_t rd_sz = Serial1.readBytesUntil('\r', uart_rx_buf, UART_RX_BUF_SIZE);
        parse_waypoint_msg(uart_rx_buf, rd_sz);
    }
    if (((waypoint_rd_idx + 1) % MAX_WAYPOINTS) != waypoint_wr_idx) {
        waypoint_rd_idx = (waypoint_rd_idx + 1) % MAX_WAYPOINTS;
        if ((waypoints[waypoint_rd_idx].x == 999) || (waypoints[waypoint_rd_idx].y == 999)) {
            rotate_indefinitely();
        }
        int16_t angle_to_target = (int16_t)(
                atan2((int16_t) waypoints[waypoint_rd_idx].y - (int16_t) current_position.y,
                        (int16_t) waypoints[waypoint_rd_idx].x - (int16_t) current_position.x) * 180.0 / 3.14159)
                - current_orientation;
        while (angle_to_target > 360) {
            angle_to_target -= 360;
        }
        while (angle_to_target < -360) {
            angle_to_target += 360;
        }
        turn_towards_target(angle_to_target, 1);
        uint16_t distance_to_target = (uint16_t) sqrt(
                pow((int16_t) waypoints[waypoint_rd_idx].x - (int16_t) current_position.x, 2)
                        + pow((int16_t) waypoints[waypoint_rd_idx].y - (int16_t) current_position.y, 2));
        move_to_target(distance_to_target);
        turn_towards_target(250, 0);
        char temp[120];
        sprintf(temp, "x: %u, y: %u, th: %d\r\n", current_position.x, current_position.y, current_orientation);
        Serial1.print(temp);
    }
}
