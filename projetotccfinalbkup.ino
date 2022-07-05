       /*********
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *********/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Arduino.h>
#include <LoRa.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_bt.h"


#define ADC_PIN 34
//RTC_DATA_ATTR aloca a variável na memoria RTC
RTC_DATA_ATTR int bootCount = 0;

//fator de conversão de microsegundos para segundos
#define uS_TO_S_FACTOR 1000000
//tempo que o ESP32 ficará em modo sleep (em segundos)
//#define TIME_TO_SLEEP 180
#define TIME_TO_SLEEP 60
//900 = 15 min

//--------------------------------------
//Define Trig and Echo pin:
#define TRIG_PIN 33 //placa caixa
#define ECHO_PIN 26
#define SENSOR_PIN 12
//--------------------------------------

//--------------------------------------
//#define TRIG_PIN 26 //teste prototipo
//#define ECHO_PIN 33
//#define SENSOR_PIN 12
//--------------------------------------

#include <WiFi.h>
#include "time.h"

const uint64_t OneMinute = 60000000LL;          // 6000000 uS = 1 minute
const uint64_t MinutesInAnHour = 1LL;    // 60 min = 1 hour 
const uint64_t SleepTimeMicroseconds = OneMinute * MinutesInAnHour * 1LL; 
//const uint64_t SleepTimeMicroseconds = OneMinute;
//esp_sleep_enable_timer_wakeup(SleepTimeMicroseconds); 


const char* ssid       = "2GHZ";
const char* password   = "369369HF";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// Define variables:
long duration;
int distance;
int dist[5];
int i,w,n, num, qtd;
int media, soma;

//Fcnt
RTC_DATA_ATTR unsigned int fcnt = 0;


static osjob_t setLowFiveSecondsJob;
static osjob_t setHighTenSecondsJob;
static osjob_t setLowJob;
static osjob_t sendjob;
static osjob_t cincoLeituras;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


static void setTrigPinToLowForFiveSeconds(osjob_t* j) {
  Serial.println("setTrigPinToLowForFiveSeconds");
  digitalWrite(TRIG_PIN, LOW);
  os_setTimedCallback(&setHighTenSecondsJob, os_getTime()+us2osticks(5), setTrigPinToHighForTenSeconds);  
}

static void setTrigPinToHighForTenSeconds(osjob_t* j) {
  Serial.println("setTrigPinToHighForTenSeconds");
  digitalWrite(TRIG_PIN, HIGH);


  os_setTimedCallback(&setLowJob, os_getTime()+us2osticks(10), setLow);  
}


static void setLow(osjob_t* j) {
   Serial.println("setLow");
  digitalWrite(TRIG_PIN, LOW);
  //digitalWrite(SENSOR_PIN, HIGH);
  duration = pulseIn(ECHO_PIN, HIGH);
          
  // Calculate the distance:
  distance = duration*0.034/2;
          
  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.println("entrando em modo sleep"); 
    
}

//END DEVICE CAIXA---------------------------------------------------------------------------------------------------------------------------------
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xA6, 0x42, 0x7E, 0x7E, 0x7F, 0x62, 0x05, 0xF4, 0x7C, 0xAF, 0x32, 0x1C, 0x55, 0x0E, 0x01, 0x98 };// caixa
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xB4, 0x6C, 0x81, 0x56, 0x3E, 0x61, 0x2D, 0xDE, 0x48, 0x15, 0xF9, 0xAA, 0xB0, 0xBD, 0xC6, 0xE8 };//caixa
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260C7EAE;//caixa
//---------------------------------------------------------------------------------------------------------------------------------------------------

//END DEVICE TESTE placa nova------------------------------------------------------------------------------------------------------------------------
//// LoRaWAN NwkSKey, network session key
//// This is the default Semtech key, which is used by the early prototype TTN
//// network.
//static const PROGMEM u1_t NWKSKEY[16] = { 0xA6, 0x42, 0x7E, 0x7E, 0x7F, 0x62, 0x05, 0xF4, 0x7C, 0xAF, 0x32, 0x1C, 0x55, 0x0E, 0x01, 0x98 };
//// LoRaWAN AppSKey, application session key
//// This is the default Semtech key, which is used by the early prototype TTN
//// network.
//static const u1_t PROGMEM APPSKEY[16] = { 0xB4, 0x6C, 0x81, 0x56, 0x3E, 0x61, 0x2D, 0xDE, 0x48, 0x15, 0xF9, 0xAA, 0xB0, 0xBD, 0xC6, 0xE8 };
//// LoRaWAN end-device address (DevAddr)
//static const u4_t DEVADDR =   0x260C7EAE;
//---------------------------------------------------------------------------------------------------------------------------------------------------

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = { 0x01, 0x02, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00 };
//static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 60;
const unsigned TX_INTERVAL = 15;
int teste = 0;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14, //2
  .dio = {2, 32, LMIC_UNUSED_PIN}, //{4, 5, 2},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:  
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
     
            //os_setTimedCallback(&setLowFiveSecondsJob, os_getTime()+sec2osticks(1), setLow_5);
              Serial.print("LMIC seqnoUp VALUE: ");Serial.println(LMIC.seqnoUp);
             
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){

      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
          //Serial.println("MEDIA dentro do dosend job");
            Serial.println(media);
             mydata[6] = (media >> 8) & 0xff;
             mydata[7] = media & 0xff; 
             
             Serial.println(mydata[6]);
             Serial.println(mydata[7]);
             
           float leituraBateria = analogRead(34)*0.0017;
           int value = leituraBateria*100;
             mydata[2] = (value >> 8) & 0xff;
             mydata[3] = value & 0xff; 
    
          Serial.println("TENSÃO DA BATERIA: ");
       //   Serial.println(analogRead(34)*3.3/4095*2);
          Serial.println(value);
             Serial.println(mydata[2]);
             Serial.println(mydata[3]);
          
        LMIC.seqnoUp =  fcnt;
        fcnt++;
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));  
        LoRa.end();
        LoRa.sleep(); 
        //digitalWrite(SENSOR_PIN, LOW);
        //esp_deep_sleep_start(); //força o ESP32 entrar em modo deep sleep     
        
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void setup() {
    
    Serial.begin(115200);
    pinMode(SENSOR_PIN, OUTPUT);
    Serial.println(F("Starting"));
    digitalWrite(SENSOR_PIN, HIGH);
    

    WiFi.persistent(false);
      
  num = 7;

  int leituras[7];
  
  for(i=0; i<=num; i++){
    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);  
    delayMicroseconds(5); 
  
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20); 
  
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.343/2;

    if(distance != 0){
    leituras[i] = distance/10;

    Serial.print("leituras:");
    Serial.println(leituras[i]);
    soma = leituras[i] + soma;
    delay(10);   
    
    }
    if(distance == 0){
    Serial.println("Leitura igual a zero, verifique sensor!");
    }
    
    delay(100);  
  }

  int aux, k, j, o; 
  for(j=0; j<=qtd; j++)
  { 
    for(k=1; k<j; k++)
    { 
      if(leituras[j] < leituras[k])
      { 
        
        aux=leituras[j]; 
        leituras[j]=leituras[k]; 
        leituras[k]=aux;
         
      } 
    } 
  } 

  qtd= 8;
  for(o=0; o<=7; o++){
    
    //Serial.print("ordenado:");
    //Serial.println(leituras[o]);   
    
      if (leituras[o] == 0){
      qtd = qtd - 1;
      }
      
    }
    
    media = soma/qtd;
    Serial.print("Media:");
    Serial.println(media); 
    
     delay(100); 

     //incrementa o numero de vezes que o BOOT ocorreu
     ++bootCount;
     Serial.println("Boot number: " + String(bootCount));

     //Print the wakeup reason for ESP32
     print_wakeup_reason();
     
     esp_sleep_enable_timer_wakeup(SleepTimeMicroseconds);
     Serial.println("Setup ESP32 to sleep for every 1 hour");
 

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);

   
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
    //setCincoLeituras(&cincoLeituras);
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
   //WiFi.forceSleepBegin();
    setCpuFrequencyMhz(80);
    adc_power_off();
   // esp_wifi_stop();
     esp_bt_controller_disable();
     esp_bluedroid_disable();

  Serial.println("Going to sleep now");
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
  
}

void loop() {
    os_runloop_once();
    printLocalTime();
}
