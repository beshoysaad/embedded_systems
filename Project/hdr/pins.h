#ifndef EMBEDDEDSYSTEMS18_PINS_H
#define EMBEDDEDSYSTEMS18_PINS_H

#include <avr/io.h>
#include <target.h>

#if TARGET == PI_ROBOT

#define SOFTSPI_CLK_DDR             DDRB
#define SOFTSPI_CLK_PORT            PORTB
#define SOFTSPI_CLK_MASK            (1 << 4)

#define SOFTSPI_MOSI_DDR            DDRB
#define SOFTSPI_MOSI_PORT           PORTB
#define SOFTSPI_MOSI_MASK           (1 << 5)

#define SOFTSPI_MISO_DDR            DDRB
#define SOFTSPI_MISO_PIN            PINB
#define SOFTSPI_MISO_MASK           (1 << 0)

#define SOFTSPI_RF_CS_DDR           DDRD
#define SOFTSPI_RF_CS_PORT          PORTD
#define SOFTSPI_RF_CS_MASK          (1 << 4)
#define SOFTSPI_RF_CS_PIN           PD4

#define RF_CE_DDR                   DDRD
#define RF_CE_PORT                  PORTD
#define RF_CE_MASK                  (1 << 7)
#define RF_CE_PIN                   PD7

#define RF_IRQ_DDR                  DDRD
#define RF_IRQ_MASK                 (1 << 2)
#define RF_IRQ_PIN                  PD2

#define ADC_SYS_CLK_DDR             DDRB
#define ADC_SYS_CLK_PORT            PORTB
#define ADC_SYS_CLK_MASK            (1 << 1)

#define SOFTSPI_ADC_CS_DDR          DDRC
#define SOFTSPI_ADC_CS_PORT         PORTC
#define SOFTSPI_ADC_CS_MASK         (1 << 5)

#elif TARGET == ZUMO_ROBOT

#define SOFTSPI_CLK_DDR             DDRB
#define SOFTSPI_CLK_PORT            PORTB
#define SOFTSPI_CLK_MASK            (1 << 0)

#define SOFTSPI_MOSI_DDR            DDRD
#define SOFTSPI_MOSI_PORT           PORTD
#define SOFTSPI_MOSI_MASK           (1 << 5)

#define SOFTSPI_MISO_DDR            DDRD
#define SOFTSPI_MISO_PIN            PIND
#define SOFTSPI_MISO_MASK           (1 << 7)

#define SOFTSPI_RF_CS_DDR           DDRB
#define SOFTSPI_RF_CS_PORT          PORTB
#define SOFTSPI_RF_CS_MASK          (1 << 3)
#define SOFTSPI_RF_CS_PIN           PB3

#define RF_CE_DDR                   DDRC
#define RF_CE_PORT                  PORTC
#define RF_CE_MASK                  (1 << 7)
#define RF_CE_PIN                   PC7

#define RF_IRQ_DDR                  DDRD
#define RF_IRQ_MASK                 (1 << 2)
#define RF_IRQ_PIN                  PD2

#endif

#endif //EMBEDDEDSYSTEMS18_PINS_H
