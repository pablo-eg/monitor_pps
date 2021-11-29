/*
*
*
*
*
*
*/


#include <EthernetENC.h>
#include "Hardware_check.h"
#include "Defines.h"
#include "Monitor.h"

Monitor mon;

void setup()
{
    mon.hardware_setup();
    mon.hardware_init();
    mon.ethernet_init();
    mon.mqtt_client_init();
}

void loop()
{
    mon.ethernet_check_connetion();
    
    #if PING_ENABLE
      mon.ping_response();
    #endif

    delay(500);
    mon.mqtt_check_connection();
    mon.mqtt_publish();

    mon.sleep(SLEEP_TIME);
}
