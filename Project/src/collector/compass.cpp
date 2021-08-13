#include <Wire.h>
#include <LSM303.h>
#include <PID_v1.h>

#define STEP_SIZE_MS                  10
#define MID_PWM                       128
#define CALIBRATION_COUNT             128
#define LEFT_MOTOR                    10
#define RIGHT_MOTOR                   9
#define LEFT_MOTOR_DIR                16
#define RIGHT_MOTOR_DIR               15
#define MOTOR_DIR_FORWARD             LOW
#define MOTOR_DIR_REVERSE             HIGH

double current_heading, pid_out, heading_bias, target_heading = 60.0;

LSM303 compass;
PID heading_pid(&current_heading, &pid_out, &target_heading, 0.5, 0.0, 0.0, DIRECT);

void calibrate_compass() {
    LSM303::vector<int16_t> running_min = { 32767, 32767, 32767 }, running_max = { -32768, -32768, -32768 };

    digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_REVERSE);
    analogWrite(RIGHT_MOTOR, MID_PWM);
    analogWrite(LEFT_MOTOR, MID_PWM);
    for (int i = 0; i < CALIBRATION_COUNT; i++) {
        compass.read();

        running_min.x = min(running_min.x, compass.m.x);
        running_min.y = min(running_min.y, compass.m.y);
        running_min.z = min(running_min.z, compass.m.z);

        running_max.x = max(running_max.x, compass.m.x);
        running_max.y = max(running_max.y, compass.m.y);
        running_max.z = max(running_max.z, compass.m.z);

        delay(10);
    }

    analogWrite(RIGHT_MOTOR, 0);
    analogWrite(LEFT_MOTOR, 0);

    compass.m_min = running_min;
    compass.m_max = running_max;

    delay(500);
}

void setup() {
    Serial1.begin(9600);
    Wire.begin();
    heading_pid.SetOutputLimits(-MID_PWM, MID_PWM);
    heading_pid.SetSampleTime(STEP_SIZE_MS);
    compass.init();
    compass.enableDefault();
    calibrate_compass();
    heading_pid.SetMode(AUTOMATIC);
}

void loop() {
    compass.read();
    current_heading = compass.heading();
    heading_pid.Compute();
    if (pid_out < 0) {
        digitalWrite(LEFT_MOTOR, MOTOR_DIR_REVERSE);
        digitalWrite(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    } else {
        digitalWrite(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        digitalWrite(RIGHT_MOTOR, MOTOR_DIR_REVERSE);
    }
    analogWrite(LEFT_MOTOR, abs(pid_out));
    analogWrite(RIGHT_MOTOR, abs(pid_out));
    Serial1.print(" pout: ");
    Serial1.print(pid_out);
    Serial1.print(" chead: ");
    Serial1.println(current_heading);
    delay(STEP_SIZE_MS);
}
