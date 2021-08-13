#include <Arduino.h>
#include <Zumo32U4LineSensors.h>
#include <Zumo32U4Motors.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
unsigned int sensors[5];
char report[128];
unsigned int num_lines;
unsigned int num_lines_detected;
bool line_detected = false;

void setup() {
    Serial1.begin(9600);
    Serial1.println("Waiting for input...");
    lineSensors.initFiveSensors();
}

void loop() {
    if (Serial1.available()) {
        num_lines = Serial1.parseInt();
        sprintf(report, "Num lines = %d\r\n", num_lines);
        Serial1.print(report);
        motors.setSpeeds(48, 48);
        while(1) {
            lineSensors.read(sensors, QTR_EMITTERS_ON);
            if ((sensors[2] > 700) && (line_detected == false)) {
                line_detected = true;
                Serial1.println("Line begin");
            } else if ((sensors[2] < 200) && (line_detected == true)) {
                line_detected = false;
                Serial1.println("Line end");
                num_lines_detected++;
                if (num_lines_detected >= num_lines) {
                    sprintf(report, "Passed %d lines\r\n", num_lines_detected);
                    Serial1.print(report);
                    motors.setSpeeds(0, 0);
                    while(1);
                }
            }
        }
    }
}