#include <PID_v1.h>
#include <light.h>
#include <3pi.h>
#include <stdio.h>
#include <string.h>

#define Light_Kp          0.5
#define Light_Ki          0.0
#define Light_Kd          0.0
#define Motion_Kp         0.75
#define Motion_Ki         0.0
#define Motion_Kd         0.0
#define SPEED             64
#define MIN_READING       100
#define STOP              0
#define OFFSET            30

double light_pid_input, light_pid_output, light_pid_setpoint, motion_pid_input, motion_pid_output, motion_pid_setpoint;

PID light_pid(&light_pid_input, &light_pid_output, &light_pid_setpoint, Light_Kp, Light_Ki, Light_Kd, DIRECT);
PID motion_pid(&motion_pid_input, &motion_pid_output, &motion_pid_setpoint, Motion_Kp, Motion_Ki, Motion_Kd, DIRECT);

int main() {
    serial_set_baud_rate(9600);
    int left_speed, right_speed;
    init_light_sensors();
    char report[64];
    uint8_t light_sensors[NUM_SENSORS];
    light_pid.SetOutputLimits(-SPEED, SPEED);
    motion_pid.SetOutputLimits(-SPEED, SPEED);
    /*Turn on PID*/
    light_pid.SetMode(AUTOMATIC);
    motion_pid.SetMode(AUTOMATIC);
    while (1) {
        read_light_sensors(light_sensors, sizeof(light_sensors));
        sprintf(report, "front %d, right %d, back %d, left %d\r\n", light_sensors[FRONT], light_sensors[RIGHT],
                light_sensors[BACK], light_sensors[LEFT]);

        light_sensors[BACK] -= (light_sensors[BACK] > OFFSET) ? OFFSET : light_sensors[BACK];

        light_pid_input = light_sensors[LEFT] - light_sensors[RIGHT];
        motion_pid_input = light_sensors[FRONT] - light_sensors[BACK];

        if ((light_sensors[LEFT] > MIN_READING) || (light_sensors[RIGHT] > MIN_READING)
                || (light_sensors[FRONT] > MIN_READING) || (light_sensors[BACK] > MIN_READING)) {
            light_pid.Compute();
            motion_pid.Compute();
            left_speed = (int) (motion_pid_output - light_pid_output);
            right_speed = (int) (motion_pid_output + light_pid_output);
            set_motors(left_speed, right_speed);
        } else {
            set_motors(STOP, STOP);
            delay(50);
            set_motors(-SPEED, SPEED);
            delay(50);
            set_motors(SPEED, -SPEED);
            delay(50);
            set_motors(STOP, STOP);
        }
        delay(20);
    }
}

