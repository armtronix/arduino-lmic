#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoRa_STM32.h>

#define BAUDRATE    115200
#define SERIALNO    42

#define ADC_PIN     PA0
#define SENDER      PB14
#define RECEIVER    PB15
#define TTN_PIN     PA8

#define OUTPUT1     PB5
#define OUTPUT2     PA12
#define OUTPUT3     PA11
#define OUTPUT4     PB9
#define OUTPUT5     PB8
#define OUTPUT6     PB1
#define OUTPUT7     PB0
#define OUTPUT8     PB10
#define OUTPUT9     PB11

int iG_Counter = 0;
float fG_AdcValue;
float fG_RefVoltage = 4.3;
float fG_BatteryVoltage;          
/*
   LoRaWAN NwkSKey, network session key.
   This is the default Semtech key, which is used by the early prototype TTN network.
 */ 
//static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/*
   LoRaWAN AppSKey, application session key.
   This is the default Semtech key, which is used by the early prototype TTN network.
 */
//static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* LoRaWAN end-device address (DevAddr). */
static const u4_t DEVADDR = 0x00000000;                                                               // <-- Change this address for every node!.

/*
   These callbacks are only used in over-the-air activation, so they are left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain).
 */
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

uint8_t myData[75];

static osjob_t sendjob;

/*
   Schedule TX every this many seconds (might become longer due to duty cycle limitations).
 */
 
const unsigned TX_INTERVAL = 60;

/* Pin mapping. */

const lmic_pinmap lmic_pins = {
  .nss = PA4,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PC13,
  .dio = {PA1, PB13, PB12},
};


void setup() 
 {
   Serial.begin(BAUDRATE);
  
   Serial.print("Board Serial Number : ");
   Serial.println(SERIALNO);
   Serial.print("Baudrate : ");
   Serial.println(BAUDRATE);   
   
   pinMode(SENDER,   INPUT);
   pinMode(RECEIVER, INPUT);
   pinMode(TTN_PIN,  INPUT);
   
   pinMode(OUTPUT1,  OUTPUT);
   pinMode(OUTPUT2,  OUTPUT);
   pinMode(OUTPUT3,  OUTPUT);
   pinMode(OUTPUT4,  OUTPUT);
   pinMode(OUTPUT5,  OUTPUT);
   pinMode(OUTPUT6,  OUTPUT);
   pinMode(OUTPUT7,  OUTPUT);
   pinMode(OUTPUT8,  OUTPUT);
   pinMode(OUTPUT9,  OUTPUT);

   while (!Serial);
   
   if(digitalRead(SENDER) == LOW)
    {
      Serial.println("LoRa SENDER MODE IS ACTIVATED");
      if(!LoRa.begin(868E6)) 
       {
         Serial.println("Starting LoRa failed!");
         while (1);
       }
    }
   else if(digitalRead(RECEIVER) == LOW)
    {
      Serial.println("LoRa RECEIVER MODE IS ACTIVATED");
      if(!LoRa.begin(868E6)) 
       {
         Serial.println("Starting LoRa failed!");
         while (1);
       }
    }
   else if(digitalRead(TTN_PIN) == LOW)
    {
      Serial.println("LoRa TTN MODE IS ACTIVATED");
      os_init();
/* Reset the MAC state. Session and pending data transfers will be discarded. */
      LMIC_reset();
/*  
   Set static session parameters. Instead of dynamically establishing a session by joining the network, 
   precomputed session parameters are be provided.
 */
      #ifdef PROGMEM
/* 
   On AVR, these values are stored in flash and only copied to RAM once. Copy them to a temporary buffer.
   here, LMIC_setSession will copy them into a buffer of its own again.
 */
        uint8_t appskey[sizeof(APPSKEY)];
        uint8_t nwkskey[sizeof(NWKSKEY)];
        memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
        memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
        LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
      #else
/* If not running an AVR with PROGMEM, just use the arrays directly. */
        LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
      #endif

      #if defined(CFG_eu868)
/* 
   Set up the channels used by the Things Network, which corresponds to the defaults of most gateways. Without this, only three base
   channels from the LoRaWAN specification are used, which certainly works, so it is good for debugging, but can overload those
   frequencies, so be sure to configure the full frequency range of your network here (unless your network autoconfigures them).
   Setting up channels should happen after LMIC_setSession, as that configures the minimal channel set.
   NA-US channels 0-71 are configured automatically.
 */ 
  
        LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);                                     // g-band.
        LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);                                     // g2-band.
      
/* 
   TTN defines an additional channel at 869.525Mhz using SF9 for class B Device's ping slots. LMIC does not have an easy way to define 
   set this frequency and support for class B is spotty and untested, so this frequency is not configured here.
 */
      #elif defined(CFG_us915)
/* 
   NA-US channels 0-71 are configured automatically but only one group of 8 should (a subband) should be active.
   TTN recommends the second sub band, 1 in a zero based count.
   https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
 */
   
        LMIC_selectSubBand(1);
      #endif
     
      LMIC_setLinkCheckMode(0);                                                                                          // Disable link check validation.
      LMIC.dn2Dr = DR_SF9;                                                                                               // TTN uses SF9 for its RX2 window.

/* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
      LMIC_setDrTxpow(DR_SF7, 14);
  
      do_send(&sendjob);                                                                                                 // Start job.   
    }
 }

void loop() 
 {
   fG_AdcValue = analogRead(ADC_PIN);
   delay(100);
   fG_BatteryVoltage = (fG_AdcValue/4096)*fG_RefVoltage;
   
   while(digitalRead(RECEIVER) == LOW)
    {
/* Try to parse packet. */
      int iL_PacketSize = LoRa.parsePacket();
      if(iL_PacketSize) 
       {
/* Received a packet. */
         Serial.print("Received Packet : ");
/* Read packet. */
         while (LoRa.available()) 
          {
            Serial.print((char)LoRa.read());
          }
         Serial.print("RSSI Value : ");
         Serial.println(LoRa.packetRssi());
         Serial.print("Signal To Noise Ratio(SNR) : ");
         Serial.print(LoRa.packetSnr());
         Serial.println(" dB");
         digitalWrite(OUTPUT1, HIGH);
         delay(1000);
         digitalWrite(OUTPUT1, LOW);
       }
    }
   while(digitalRead(SENDER) == LOW)
    {
      fG_AdcValue = analogRead(ADC_PIN);
      Serial.println(fG_AdcValue);
      delay(100);
      fG_BatteryVoltage = (fG_AdcValue/4096)*fG_RefVoltage;
      Serial.print("Sending Packet: ");
      Serial.println(iG_Counter);

/* Send packet. */
      LoRa.beginPacket();
      LoRa.print("Welcome To ARMtronix Technologies : ");
      LoRa.println(iG_Counter);
      LoRa.print("Battery Voltage : ");
      LoRa.println(fG_BatteryVoltage);
      LoRa.endPacket();
      iG_Counter++;
      Serial.print("Battery Voltage : ");
      Serial.println(fG_BatteryVoltage);
      digitalWrite(OUTPUT9, HIGH);
      delay(1000);
      digitalWrite(OUTPUT9, LOW);
      delay(5000);
    }
   while(digitalRead(TTN_PIN) == LOW)
    {
      os_runloop_once();
      digitalWrite(OUTPUT1, HIGH);
      delay(100);      
      digitalWrite(OUTPUT2, HIGH);
      delay(100);
      digitalWrite(OUTPUT3, HIGH);
      delay(100);
      digitalWrite(OUTPUT4, HIGH);
      delay(100);
      digitalWrite(OUTPUT5, HIGH);
      delay(100);
      digitalWrite(OUTPUT6, HIGH);
      delay(100);
      digitalWrite(OUTPUT7, HIGH);
      delay(100);
      digitalWrite(OUTPUT8, HIGH);
      delay(100);
      digitalWrite(OUTPUT9, HIGH);
      delay(100);      
      digitalWrite(OUTPUT1, LOW);
      delay(100);      
      digitalWrite(OUTPUT2, LOW);
      delay(100);
      digitalWrite(OUTPUT3, LOW);
      delay(100);
      digitalWrite(OUTPUT4, LOW);
      delay(100);
      digitalWrite(OUTPUT5, LOW);
      delay(100);
      digitalWrite(OUTPUT6, LOW);
      delay(100);
      digitalWrite(OUTPUT7, LOW);
      delay(100);
      digitalWrite(OUTPUT8, LOW);
      delay(100);
      digitalWrite(OUTPUT9, LOW);                              
    }
 }

 /*##############################################################################################################################################################################*/
void onEvent (ev_t ev) 
 {
   // Serial.print(os_getTime());
   // Serial.print(": ");
   switch (ev) 
    {
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
           //Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
           if(LMIC.txrxFlags & TXRX_ACK)
           Serial.println(F("Received ack"));
           if(LMIC.dataLen) 
            {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }      
           os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);                              // Schedule next transmission.
           break;
      case EV_LOST_TSYNC:
           Serial.println(F("EV_LOST_TSYNC"));
           break;
      case EV_RESET:
           Serial.println(F("EV_RESET"));
           break;
      case EV_RXCOMPLETE:
           Serial.println(F("EV_RXCOMPLETE"));                                                                           // Data received in ping slot.
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

/*##############################################################################################################################################################################*/
bool prepareDataTransmission() 
 {
   Serial.println("Message Sent : Welcome To ARMtronix");  
   uint8_t message[] = "Welcome To ARMtronix";
   LMIC_setTxData2(1, message, sizeof(message) - 1, 0);
   return true;
 }

/*##############################################################################################################################################################################*/
void do_send(osjob_t* j) 
 {
/* Check if there is not a current TX/RX job running. */
   if(LMIC.opmode & OP_TXRXPEND) 
    {
      Serial.println(F("OP_TXRXPEND, not sending"));
    } 
   else 
    {
      prepareDataTransmission();
      //delay(100);
    }
/* Next TX is scheduled after TX_COMPLETE event. */
 }
/*##############################################################################################################################################################################*/
