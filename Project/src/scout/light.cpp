#include <pins.h>
#include <avr/io.h>
#include <softspi.h>
#include <time.h>
#include <stdint.h>

static void config_sys_clk(void) {
    ADC_SYS_CLK_DDR |= ADC_SYS_CLK_MASK;
    ADC_SYS_CLK_PORT &= ~ADC_SYS_CLK_MASK;
    TCCR1A |= (1 << COM1A0); // Toggle OC1A (PB1) on compare match
    TCCR1B |= (1 << WGM12); // CTC mode
    OCR1A = 9; // Produce a 1 MHz clock signal
    TCCR1B |= (1 << CS10); // Start timer. No prescaling
}

void init_light_sensors(void) {
    SOFTSPI_ADC_CS_DDR |= SOFTSPI_ADC_CS_MASK;
    SOFTSPI_ADC_CS_PORT |= SOFTSPI_ADC_CS_MASK;
    config_sys_clk();
    softspi_setup_master();
}

void read_light_sensors(uint8_t *sensors, uint8_t len) {
    SOFTSPI_ADC_CS_PORT &= ~SOFTSPI_ADC_CS_MASK;
    delay_us(4);
    uint8_t addr = 0;
    softspi_write(&addr, 1);
    for (uint8_t i = 1; i < len + 1; i++) {
        SOFTSPI_ADC_CS_PORT |= SOFTSPI_ADC_CS_MASK;
        delay_us(40);
        SOFTSPI_ADC_CS_PORT &= ~SOFTSPI_ADC_CS_MASK;
        addr = i << 4;
        softspi_transfer(&addr, 1, &sensors[i - 1], 1);
    }
    SOFTSPI_ADC_CS_PORT |= SOFTSPI_ADC_CS_MASK;
}
