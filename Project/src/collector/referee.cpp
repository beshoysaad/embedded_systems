#include <Arduino.h>
#include <radio.h>
#include <addresses.h>

char uart_rx_buf[128];
unsigned theta, x, y;
int theta_in, x_in, y_in;
bool flag=false;
radiopacket_t test;

void radio_rxhandler(uint8_t pipe_number){

}

void setup(){
    Serial1.begin(9600);
    Serial1.setTimeout(20000);
    Serial1.print("\r\n\t- Referee Simulator -\t\r\n");

    Radio_Init();
   // attachInterrupt(2, RF_rx_ISR, FALLING);

    uint8_t own_address[] = REFEREE_ADDRESS;

    uint8_t dist_address[] = SCOUT_ADDRESS;

    Radio_Configure_Rx(RADIO_PIPE_0, own_address, ENABLE);

    Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);

    Radio_Set_Tx_Addr(dist_address);


    }



void loop() {
    if ((Serial1.available()) ) {



        size_t s = Serial1.readBytesUntil('\r', uart_rx_buf, sizeof(uart_rx_buf));
       // Serial1.print(s);
       if (s > 5) {
            sscanf(uart_rx_buf, "%d %d %d\r\n", &theta_in, &x_in, &y_in);
            theta = (unsigned) ((double) theta_in * 65535.0 / 360.0);
            x = x_in * 10;
            y = y_in * 10;
            test.payload[0] = 0x61;
            test.payload[1] = (theta >> 8) & 0xFF;
            test.payload[2] = theta & 0xFF;
            test.payload[3] = (x >> 8) & 0xFF;
            test.payload[4] = x & 0xFF;
            test.payload[5] = (y >> 8) & 0xFF;
            test.payload[6] = y & 0xFF;
            uint8_t ret_val_tx = Radio_Transmit(&test, 7, RADIO_RETURN_ON_TX);
            flag = true;
            if (ret_val_tx != RADIO_TX_SUCCESS) {
                Serial1.println("Tx Failure");
            }
            else{
                Serial1.print("Done");

            }
            memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
        }
    }

}