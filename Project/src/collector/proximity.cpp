#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors Motor;

void TurnLeft() {

    Motor.setSpeeds(100, -100);

}

void TurnRight() {

    Motor.setSpeeds(-100, 100);

}

void MoveForward() {

    Motor.setSpeeds(100, 100);
}

void StopMoving() {

    Motor.setSpeeds(0, 0);
}

void setup() {
    Serial1.begin(9600);
    Serial1.println("say hi");
    Serial1.println("LL\tFL\tFR\tRR");
    proxSensors.initThreeSensors();
}

void loop() {
    proxSensors.read();

    int LeftLeft = proxSensors.countsLeftWithLeftLeds();
    int FrontLeft = proxSensors.countsFrontWithLeftLeds();
    int FrontRight = proxSensors.countsFrontWithRightLeds();
    int RightRight = proxSensors.countsRightWithRightLeds();

    Serial1.print(LeftLeft);
    Serial1.print("\t");
    Serial1.print(FrontLeft);
    Serial1.print("\t");
    Serial1.print(FrontRight);
    Serial1.print("\t");
    Serial1.print(RightRight);
    Serial1.print("\t");
    Serial1.println("");
    if (FrontLeft == FrontRight) {
        MoveForward();
        delay(500);
        StopMoving();
    }
    if (LeftLeft > 0 || RightRight > 0) {
        if (LeftLeft > 0) {
            if (FrontLeft >= FrontRight) {
                TurnLeft();
                delay(100);
            }
            MoveForward();
            delay(500);
            StopMoving();
        };
        if (RightRight > 0) {
            if (FrontRight >= FrontLeft) {
                TurnRight();
                delay(500);
            }
        };
    }
    /* does not see anything, moving right, front and left until finding.   */

    if (LeftLeft == 0 && RightRight == 0 && FrontLeft == 0 && FrontRight == 0) {
        MoveForward();
        delay(100);

        TurnLeft();
        delay(200);
        MoveForward();
        delay(100);

        TurnRight();
        delay(200);
        MoveForward();
        delay(100);
    }

    delay(100);
}
