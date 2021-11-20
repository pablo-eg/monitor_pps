#ifndef DEFINES_H
    #define DEFINES_H

    #define MONITOR_LOGLEVEL 3

    // SS SPI pin
    #define SS_PIN          PA4

    #define ETHERNET_LED    PC13

    #define PING_ENABLE     1

    #define DHTPIN          PB5
    #define DHTTYPE         DHT11
    #define PUBLISH_TIME    15000    // en ms

    #define INPUT_VAC       PA0
    #define INPUT_IAC       PA1
    #define OUTPUT_VAC      PA2
    #define OUTPUT_IAC      PA3
    #define SAMPLES         80      // Ciclos sampleados = SAMPLES * 50Hz/SAMPLES_FREQ      
    #define SAMPLE_FREQ     1000    // en Hz
    #define CONVERSION_VAC  1.899   // determinado empiricamente 226/119
    #define CONVERSION_IAC  27.99   // determinado empiricamente 1000*3.163/113
    #define CURRENT_NOISE   50.0    // Valores por debajo de los 50 mA se consideran 0 mA
    #define VOLTAGE_NOISE   100.0   // Valores por debajo de los 100 V se consideran 0 V

    #define SLEEP_TIME      15000   // en milisegundos
#endif
