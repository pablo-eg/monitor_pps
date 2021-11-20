#ifndef MONITOR_H
    #define MONITOR_H

    #include <Arduino.h>
    #include "Defines.h"
    #include <EthernetENC.h>
    #include <EthernetUdp.h>
    #include <PubSubClient.h>
    #include <DHT.h>
    #include <Wire.h>
    #include <Adafruit_INA219.h>
    #include "STM32SimpleRTC.h"
    #include "STM32SimpleLowPower.h"


    class Monitor
    {
        private:
            bool ethernet_connected = false;
            bool mqtt_connected = false;
            void calibra_sensores(void);
            
        public:
            Monitor();
            
            void hardware_setup(void);
            void hardware_init(void);
            void enc28j60_power_on(void);
            void enc28j60_power_off(void);
            void ethernet_init(void);
            void ethernet_check_connetion(void);
            
            #if PING_ENABLE
                void ping_response(void);
            #endif

            void mqtt_client_init(void);
            void mqtt_check_connection(void);
            void mqtt_publish(void);

            void sleep(unsigned long);
            //void check_temp_hum(void);
            //void check_dc_power(int sensor_id);
    };    

#endif 
