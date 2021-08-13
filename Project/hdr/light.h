#ifndef EMBEDDEDSYSTEMS18_LIGHT_H
#define EMBEDDEDSYSTEMS18_LIGHT_H

#include <stdint.h>

typedef enum {
    FRONT, RIGHT, BACK, LEFT, NUM_SENSORS,
} sensor_t;

void init_light_sensors(void);
void read_light_sensors(uint8_t *sensors, uint8_t len);

#endif //EMBEDDEDSYSTEMS18_LIGHT_H
