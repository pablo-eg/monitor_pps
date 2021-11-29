#include "Monitor.h"


IPAddress actualIP;
// monitor MAC
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// monitor IP
IPAddress ip(172, 17, 200, 57);


#if PING_ENABLE
    unsigned int localPort = 8888;
    EthernetUDP Udp;
#endif

EthernetClient ethClient;
// objeto cliente mqtt
PubSubClient client(ethClient);
// broker IP
IPAddress broker(172, 16, 19, 101);

// objeto dht (sensor de temperatura)
DHT dht(DHTPIN, DHTTYPE);

// odjeto sensor corriente/tension de la bateria y el panel
Adafruit_INA219 ina_battery(0x40);
Adafruit_INA219 ina_panel(0x41);

/************* TIMER SETUP *************/
#if defined(TIM1)
    TIM_TypeDef *Instance = TIM1;
#else
    TIM_TypeDef *Instance = TIM2;
#endif
HardwareTimer *MyTim = new HardwareTimer(Instance);

// los buffers son vectores que contienen las muestras
int input_vac_buffer[SAMPLES];
int input_iac_buffer[SAMPLES];
int output_vac_buffer[SAMPLES];
int output_iac_buffer[SAMPLES];
bool buffers_filled = false;

// los valores de los offset son calculados en la calibración 
int input_vac_offset  = 512;
int output_vac_offset = 512;
int input_iac_offset  = 500;
int output_iac_offset = 500;

// cuando se expira el timer del muestreo se llama a esta funcion
void timer_callback(void);

Monitor::Monitor()
{

}

// calcula los offset de los sensores tomando SAMPLES muestras y haciendo un promedio
void Monitor::calibra_sensores()
{
    #if(MONITOR_LOGLEVEL>0)
        Serial.println();
        Serial.println("Calibration of sensors...");
    #endif

    int i;
    int32_t input_vac_sum  = 0;
    int32_t output_vac_sum = 0;
    int32_t input_iac_sum  = 0;
    int32_t output_iac_sum = 0;

    for(i=0; i<SAMPLES; i++)
    {
        input_vac_buffer[i]  = analogRead(INPUT_VAC);
        input_iac_buffer[i]  = analogRead(INPUT_IAC);
        output_vac_buffer[i] = analogRead(OUTPUT_VAC);
        output_iac_buffer[i] = analogRead(OUTPUT_IAC);
    }

    for(i=0; i<SAMPLES; i++)
    {
        input_vac_sum  += input_vac_buffer[i];
        output_vac_sum += output_vac_buffer[i];
        input_iac_sum  += input_iac_buffer[i];
        output_iac_sum += output_iac_buffer[i];
    }

    input_vac_offset  = input_vac_sum/SAMPLES;
    output_vac_offset = output_vac_sum/SAMPLES;
    input_iac_offset  = input_iac_sum/SAMPLES;
    output_iac_offset = output_iac_sum/SAMPLES; 

    #if(MONITOR_LOGLEVEL>0)
        Serial.print("input_vac_offset: ");
        Serial.println(input_vac_offset);
        Serial.print("output_vac_offset: ");
        Serial.println(output_vac_offset);
        Serial.print("input_iac_offset: ");
        Serial.println(input_iac_offset);
        Serial.print("output_iac_offset: ");
        Serial.println(output_iac_offset);
    #endif 
}

// setea algunos pines y la frecuencia del timer de muestreo
void Monitor::hardware_setup()
{
    pinMode(ETHERNET_LED, OUTPUT); 

    /************* TIMER SETUP *************/
    MyTim->setOverflow(SAMPLE_FREQ, HERTZ_FORMAT);
    MyTim->attachInterrupt(timer_callback);
}

// inicializa los sensores
void Monitor::hardware_init()
{
    digitalWrite(ETHERNET_LED, LOW);        // tener en cuenta que la logica en este led esta invertida

    STM32SimpleLowPower::begin();
    
    #if(MONITOR_LOGLEVEL>0)
        Serial.begin(9600);
        delay(100);
        for(int i=0; i<10; i++) Serial.println();
        Serial.println("************* MONITOR DEBUGGING ************");
    #endif

    #if PING_ENABLE
        Udp.begin(localPort);
    #endif

    dht.begin();

    if(!ina_battery.begin())
    {
        #if(MONITOR_LOGLEVEL>0)
            Serial.println("Failed to find INA219 chip (baterry)");
        #endif
    }
    ina_battery.setCalibration_16V_400mA();

    if(!ina_panel.begin())
    {
        #if(MONITOR_LOGLEVEL>0)
            Serial.println("Failed to find INA219 chip (solar panel)");
        #endif
    }
    ina_panel.setCalibration_16V_400mA();

    calibra_sensores();
}

// despierta al modulo ethernet. Cuando esta activo consume 120 mA aprox
void Monitor::enc28j60_power_on(void)
{
    #if(MONITOR_LOGLEVEL>0)
        Serial.println(" ");
        Serial.println("Powering on enc28j60...");
    #endif

    Enc28J60Network::powerOn();
}

// manda a dormir al modulo ethernet. Cuando esta inactivo consume menos de 1 mA
void Monitor::enc28j60_power_off(void)
{
    #if(MONITOR_LOGLEVEL>0)
        Serial.println(" ");
        Serial.println("Powering off enc28j60...");
    #endif

    Enc28J60Network::powerOff();
}

// intenta conectarse por Ethernet con la IP seteada
void Monitor::ethernet_init()
{
    Ethernet.init(SS_PIN);

    #if(MONITOR_LOGLEVEL>0)
        Serial.println(" ");
        Serial.println("Connecting to ethernet...");
    #endif

    Ethernet.begin(mac, ip);
    delay(500);
    actualIP = Ethernet.localIP();

    if((Ethernet.linkStatus() == LinkON) && ((uint32_t)actualIP != 0))
    {
        digitalWrite(ETHERNET_LED, HIGH);       // cuando esta conectado por ethernet se apaga el led
        ethernet_connected = true;

        #if(MONITOR_LOGLEVEL>0)
            Serial.println();
            Serial.println("Connected :)");
            Serial.print("IP: ");
            Serial.println(actualIP);
            Serial.print("Mask: ");
            Serial.println(Ethernet.subnetMask());     
            Serial.print("Gateway: ");
            Serial.println(Ethernet.gatewayIP());
            Serial.print("DNS: ");
            Serial.println(Ethernet.dnsServerIP());
        #endif
    }
    else
    {   
        ethernet_connected = false;

        #if (MONITOR_LOGLEVEL>0)
            Serial.println();
            Serial.println("Error :(");
        #endif
    }
}

// si ha pasado cierto tiempo chequea la conexión ethernet e intenta conectarse
void Monitor::ethernet_check_connetion()
{
    const unsigned long intervalo = 5000;     // intervalo de tiempo en el que realiza el checkeo
    unsigned long tiempo_actual = millis();
    static unsigned long timeout = intervalo;
    int linkStatus = 0;

    if(tiempo_actual > timeout)         // cada <intervalo> tiempo se checkea la conexion
    {
        timeout = tiempo_actual + intervalo;

        actualIP = Ethernet.localIP();

        if((Ethernet.linkStatus() == LinkON) && ((uint32_t)actualIP != 0))
        {
            digitalWrite(ETHERNET_LED, HIGH);   
            
            #if(MONITOR_LOGLEVEL>0)
                Serial.println();
                Serial.println("Link status: up");
            #endif

            ethernet_connected = true;
        }
        else
        {
            digitalWrite(ETHERNET_LED, LOW);       // si hay algun problema con la conexion se enciende el buildinled
            
            #if(MONITOR_LOGLEVEL>0)
                Serial.println();
                Serial.println("Link status: down");
            #endif

            ethernet_connected = false;
        }
    }
}

#if PING_ENABLE
// para que responda ping tuve que habilitar el procotoco UDP
// Buscar otras alternativas luego
    void Monitor::ping_response()
    {
        int packetSize = Udp.parsePacket();

        delay(10);
    }
#endif

// cuando se recibe un mensaje MQTT se llama a esta funcion
void callback(char* topic, byte* payload, unsigned int length)
{
    char buffer[20];
    snprintf(buffer, length+1, "%s", payload);
    
    #if(MONITOR_LOGLEVEL>0)
        Serial.print("Message arrived [");
        Serial.print(topic);
        Serial.print("]: ");
        Serial.println(buffer);
    #endif

    if(strcmp(buffer, "monitor_shutdown") == 0) // are equal
    {
        #if(MONITOR_LOGLEVEL>0)
        Serial.print("Turning off monitor for ");
        Serial.print(SLEEP_TIME/1000);
        Serial.println(" seconds...bye");
        #endif
        
        client.disconnect();
        delay(1000);
        Enc28J60Network::powerOff();
        delay(SLEEP_TIME);
        Enc28J60Network::powerOn();
        NVIC_SystemReset();
    }
}

// setea el cliente MQTT
void Monitor::mqtt_client_init()
{
    //client.setServer("broker.emqx.io", 1883);
    client.setServer(broker, 1883);
    client.setCallback(callback);
}   

// checkea la conexion con el broker
void Monitor::mqtt_check_connection()
{
    if(ethernet_connected)
    {
        client.loop();

        if(!client.connected())
        {
            //const unsigned long intervalo = 5000;     // intervalo de tiempo en el que realiza el checkeo
            const unsigned long intervalo = 0;     // intervalo de tiempo en el que realiza el checkeo
            unsigned long tiempo_actual = millis();
            static unsigned long timeout = intervalo;

            if(tiempo_actual > timeout)         // cada <intervalo> tiempo se checkea la conexion
            {
                timeout = tiempo_actual + intervalo;

                #if(MONITOR_LOGLEVEL>0)
                    Serial.println();
                    Serial.print("Attempting MQTT connection...");
                #endif 
                if (client.connect("arduinoClient"))
                {
                    #if(MONITOR_LOGLEVEL>0)
                        Serial.println("connected");
                    #endif
                    // Once connected, publish an announcement...
                    client.publish("outTopic","hello world");
                    // ... and resubscribe
                    client.subscribe("inTopic");

                    mqtt_connected = true;
                }
                else
                {
                    mqtt_connected = false;
                    
                    #if(MONITOR_LOGLEVEL>0)
                        Serial.print("failed, rc=");
                        Serial.print(client.state());
                        Serial.println(" trying again");
                    #endif
                    
                    for(int i=0; i<20; i++)
                    {
                        if (client.connect("arduinoClient"))
                        {
                            #if(MONITOR_LOGLEVEL>0)
                                Serial.println("connected");
                            #endif

                            mqtt_connected = true;
                            break;
                        }
                        else
                        {
                            #if(MONITOR_LOGLEVEL>0)
                                Serial.println("failed...trying again");
                            #endif
                        }
                    }

                    
                }
            }
        }
    }

}

// publica en el broker las mediciones
void Monitor::mqtt_publish()
{
    if(mqtt_connected)
    {
        const unsigned long intervalo = PUBLISH_TIME;     // intervalo de tiempo en el que realiza el checkeo
        unsigned long tiempo_actual = millis();
        static unsigned long timeout = intervalo;
        char result[8];

        //if(tiempo_actual > timeout)         // cada <intervalo> tiempo se checkea la conexion
        if(1)
        {
            timeout = tiempo_actual + intervalo;
            
            // Medir humedad
            float h = dht.readHumidity();
            dtostrf(h, 4, 2, result);
            client.publish("monitor/humedad", result);

            // Medir temperatura
            float t = dht.readTemperature();
            dtostrf(t, 4, 2, result);
            client.publish("monitor/temperatura", result);

            // Medir potencia en la bateria
            float shuntvoltage = ina_battery.getShuntVoltage_mV();
            float busvoltage = ina_battery.getBusVoltage_V();
            float current_mA = ina_battery.getCurrent_mA();
            
            float loadvoltage = busvoltage + (shuntvoltage / 1000);

            dtostrf(loadvoltage, 4, 2, result);
            client.publish("monitor/vbat", result);

            dtostrf(current_mA, 4, 2, result);
            client.publish("monitor/ibat", result);

            // Medir potencia en el panel
            shuntvoltage = ina_panel.getShuntVoltage_mV();
            busvoltage = ina_panel.getBusVoltage_V();
            current_mA = ina_panel.getCurrent_mA();
            
            loadvoltage = busvoltage + (shuntvoltage / 1000);

            dtostrf(loadvoltage, 4, 2, result);
            client.publish("monitor/vpanel", result);

            dtostrf(current_mA, 4, 2, result);
            client.publish("monitor/ipanel", result);

            MyTim->resume();
            while(!buffers_filled) delay(5);
            

            buffers_filled = false;
            
            /************ RMS CALC ************/
            int32_t input_vac_sum_squ = 0;
            int32_t output_vac_sum_squ = 0;
            int32_t input_iac_sum_squ = 0;
            int32_t output_iac_sum_squ = 0;

            for(int i=0; i<SAMPLES; i++)
            {
                input_vac_buffer[i] -= input_vac_offset;
                input_vac_sum_squ += (input_vac_buffer[i]*input_vac_buffer[i]);

                output_vac_buffer[i] -= output_vac_offset;
                output_vac_sum_squ += (output_vac_buffer[i]*output_vac_buffer[i]);

                input_iac_buffer[i] -= input_iac_offset;
                input_iac_sum_squ += (input_iac_buffer[i]*input_iac_buffer[i]);

                output_iac_buffer[i] -= output_iac_offset;
                output_iac_sum_squ += (output_iac_buffer[i]*output_iac_buffer[i]);
            }

            float input_vac  = sqrt(input_vac_sum_squ/SAMPLES)  * CONVERSION_VAC;
            float output_vac = sqrt(output_vac_sum_squ/SAMPLES) * CONVERSION_VAC;
            float input_iac  = sqrt(input_iac_sum_squ/SAMPLES)  * CONVERSION_IAC;
            float output_iac = sqrt(output_iac_sum_squ/SAMPLES) * CONVERSION_IAC;
            
            if(input_vac>VOLTAGE_NOISE)
            {
                dtostrf(input_vac, 4, 2, result);
                client.publish("monitor/input_vac", result);
            }
            else
            {
                client.publish("monitor/input_vac", "0");
            }

            if(output_vac>VOLTAGE_NOISE)
            {
                dtostrf(output_vac, 4, 2, result);
                client.publish("monitor/output_vac", result);
            }
            else
            {
                client.publish("monitor/output_vac", "0");
            }

            if(input_iac>CURRENT_NOISE)
            {
                dtostrf(input_iac, 4, 2, result);
                client.publish("monitor/input_iac", result);
            }
            else
            {
                client.publish("monitor/input_iac", "0");
            }

            if(output_iac>CURRENT_NOISE)
            {
                dtostrf(output_iac, 4, 2, result);
                client.publish("monitor/output_iac", result);
            }
            else
            {
                client.publish("monitor/output_iac", "0");
            }
            

            #if(MONITOR_LOGLEVEL>0)
                Serial.println("Topics published");
            #endif
        }
    }
}

// cuando expira el timer del muestreo se llama a esta funcion para que tome una nueva muestra
void timer_callback()
{
    static int i = 0;

    input_vac_buffer[i]  = analogRead(INPUT_VAC);
    input_iac_buffer[i]  = analogRead(INPUT_IAC);
    output_vac_buffer[i] = analogRead(OUTPUT_VAC);
    output_iac_buffer[i] = analogRead(OUTPUT_IAC);
 
    i++;
    if(i==SAMPLES)
    {
        i = 0;
        buffers_filled = true;
        MyTim->pause();         // cuando se completa el total de muestras, se pausa el timer
    }
}

// manda a dormir al sistema logrando un consumos menor al miliamper
void Monitor::sleep(unsigned long sleep_time)
{
        unsigned long t = sleep_time;
        client.disconnect();
        delay(1000);
        Enc28J60Network::powerOff();
        delay(1000);
        STM32SimpleLowPower::deepSleep(20);
        delay(1000);
        Enc28J60Network::powerOn();
        delay(1000);
        NVIC_SystemReset();
        
}