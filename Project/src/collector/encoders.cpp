#include <Arduino.h>
#include <Zumo32U4Encoders.h>

#define WHEEL_RADIUS                  2.0
#define COLLECTOR_DIMENSION           8.5
#define LEFT_MOTOR                    10
#define RIGHT_MOTOR                   9
#define LEFT_MOTOR_DIR                16
#define RIGHT_MOTOR_DIR               15
#define MOTOR_DIR_FORWARD             LOW
#define MOTOR_DIR_REVERSE             HIGH
#define ANGLE_CORRECTION_FACTOR       40
#define DISTANCE_CORRECTION_FACTOR    50

double current_theta, current_x, current_y;
unsigned long current_time_ms, last_time_ms;
Zumo32U4Encoders encoders;

void setup() {
    Serial1.begin(9600);
    Serial1.println("\r\n\t- Program Start -\t\r\n");
    encoders.init();
    digitalWrite(LEFT_MOTOR_DIR, MOTOR_DIR_FORWARD);
    digitalWrite(RIGHT_MOTOR_DIR, MOTOR_DIR_REVERSE);
    analogWrite(RIGHT_MOTOR, 255);
    analogWrite(LEFT_MOTOR, 255);
}

void loop() {
    current_time_ms = millis();
    double delta_ms = (double) (current_time_ms - last_time_ms);
    if (delta_ms != 0) {
        last_time_ms = current_time_ms;
        double angular_l = ((double) encoders.getCountsAndResetLeft()) * 1000.0 * 60.0 * 0.10472 / (delta_ms * 909.7);
        double angular_r = ((double) encoders.getCountsAndResetRight()) * 1000.0 * 60.0 * 0.10472 / (delta_ms * 909.7);
        current_theta += WHEEL_RADIUS * (angular_r - angular_l) / (COLLECTOR_DIMENSION * ANGLE_CORRECTION_FACTOR);
        current_x += WHEEL_RADIUS * (angular_r + angular_l) * cos(current_theta) / (2.0 * DISTANCE_CORRECTION_FACTOR);
        current_y += WHEEL_RADIUS * (angular_r + angular_l) * sin(current_theta) / (2.0 * DISTANCE_CORRECTION_FACTOR);
        Serial1.print("th: ");
        Serial1.print(current_theta * 180 / 3.14159);
        Serial1.print(" x: ");
        Serial1.print(current_x);
        Serial1.print(" y: ");
        Serial1.println(current_y);
        if (current_time_ms >= 100) {
            analogWrite(RIGHT_MOTOR, 0);
            analogWrite(LEFT_MOTOR, 0);
            while (1)
                ;
        }
    }
}
