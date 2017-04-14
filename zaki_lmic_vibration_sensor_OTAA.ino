/*******************************************************************************
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
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * 29 December 2016 Modified by Zaki to cater for RF95 + Arduino 
 *******************************************************************************/

#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <avr/sleep.h>


volatile float averageVcc = 0.0;
int wakePin = 3;                 // pin used for waking up. I attach vibration sensor SW-420 and set to the highest sensitivity.
int count=0;


/************************************* OTAA Section : must fill these ******************************************************************/
/* This EUI must be in little-endian format, so least-significant-byte
   first. When copying an EUI from ttnctl output, this means to reverse
   the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70. */
   
static const u1_t PROGMEM APPEUI[8]={ 0x60, 0x45, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };
/*Eg : 70B3D57EF0004560 as got from TTN dashboard*/
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

/* This should also be in little endian format, see above. */
static const u1_t PROGMEM DEVEUI[8]={ 0x78, 0xD3, 0xDC, 0x03, 0x04, 0x73, 0x80, 0x00 };
/*Eg : 0080730403DCD378 as got from TTN dashboard */
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

/* This key should be in big endian format (or, since it is not really a
   number but a block of memory, endianness does not really apply). In
   practice, a key taken from ttnctl can be copied as-is.
   The key shown here is the semtech default key. */
static const u1_t PROGMEM APPKEY[16] = { 0x46, 0x09, 0xE8, 0xF7, 0x0E, 0x32, 0x1B, 0x0C, 0x6C, 0xCD, 0x21, 0x04, 0x4B, 0x03, 0xC1, 0x8A };
/*Eg : 4609E8F70E321B0C6CCD21044B03C18A as got from TTN dashboard */
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

/**************************************** OTAA Section Ends *********************************************************************************/


/********************************* ABP Section : Need to fill these *************************************************************************/
/* LoRaWAN NwkSKey, network session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
//static const PROGMEM u1_t NWKSKEY[16] = { 0xD2, 0x9E, 0xA7, 0x33, 0xC5, 0xC5, 0xFF, 0x7A, 0xCE, 0x85, 0xFC, 0x5F, 0x62, 0xAA, 0xBB, 0x5D };
/*Eg : D29EA733C5C5FF7ACE85FC5F62AABB5D as got from TTN dashboard*/

/* LoRaWAN AppSKey, application session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
//static const u1_t PROGMEM APPSKEY[16] = { 0xD0, 0xF1, 0x30, 0xAC, 0xC7, 0x33, 0x06, 0xB1, 0xF6, 0x7A, 0xBF, 0x92, 0xEF, 0x1A, 0xCD, 0x0F };
/*Eg : D0F130ACC73306B1F67ABF92EF1ACD0F as got from TTN dashboard */

/* LoRaWAN end-device address (DevAddr)
   See http://thethingsnetwork.org/wiki/AddressSpace */
//static const u4_t DEVADDR = 0x260219FF ; // <-- Change this address for every node!
/*Eg : 260219FF as got from TTN dashboard */

/* These callbacks are only used in over-the-air activation, so they are
   left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain). */
   
//void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
//void os_getDevKey (u1_t* buf) { }

/********************************************* ABP Section Ends ***************************************************************************/


//static uint8_t mydata[] = "Hello, mine is ABP!";
static osjob_t sendjob;


/* Schedule TX every this many seconds (might become longer due to duty
   cycle limitations). */
const unsigned TX_INTERVAL = 10;

/* Pin mapping */
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
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

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            
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
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            /* data received in ping slot */
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
    
    static uint8_t message[2];
    
    /* Check if there is not a current TX/RX job running */
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /* Prepare upstream data transmission at the next possible time. */

        count++;
        Serial.println(" ");   
        if(count > 10)
        {
           count = 1;
           delay(2000);
           LMIC_setSleep();
           sleepNow();     // sleep function called here
        }  


        /****** Send "Hello, Mine is ABP" ******/
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        /**************************************/
        
        /******* Proximity Sensor *************/
        //Serial.print("\nDistance: ");
        //uint8_t Distance = sonar.ping_cm();
        //Serial.print(Distance);
        //Serial.println("cm");
        //message[0] = Distance;
        /**************************************/
        
        /******* Voltage Sensor ***************/
        averageVcc = averageVcc + (float) readVcc();
        int16_t averageVccInt = round(averageVcc);
        Serial.print(averageVccInt);
        Serial.print(" mV \n"); 
        message[0] = highByte(averageVccInt);
        message[1] = lowByte(averageVccInt);
        /**************************************/
        
        LMIC_setTxData2(1, message, sizeof(message), 0);        
        Serial.println(F("Packet queued"));
        /*Print Freq being used*/
        Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);
        /*Print mV being supplied*/
        
        delay(1000);
        averageVcc = 0;
                
    }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));   

    pinMode(wakePin, INPUT);  
    attachInterrupt(1, wakeUpNow, RISING); // use interrupt 1 (pin 3) and run function wakeUpNow when pin 3 gets RISING
                                           // once vibration is detected, the rising of the pin signal will wake the board.  
    LoraInitialization();  // Do all Lora Init Stuff

    /* Start job */
    do_send(&sendjob);
}

void loop() {   
    os_runloop_once();   
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void sleepNow() {  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here  
    sleep_enable();          // enables the sleep bit in the mcucr register  
    attachInterrupt(1,wakeUpNow, RISING); // use interrupt 1 (pin 3) and run function  
    sleep_mode();            // here the device is actually put to sleep!!  
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
    sleep_disable();         // first thing after waking from sleep: disable sleep...  
    detachInterrupt(1);      // disables interrupt 1 on pin 3 so the wakeUpNow code will not be executed during normal running time. 
    LMIC_setStandby();
    LoraInitialization();
}  

void wakeUpNow() {  
  // execute code here after wake-up before returning to the loop() function  
  // timers and code using timers (serial.print and more...) will not work here.  
  // we don't really need to execute any special functions here, since we  
  // just want the thing to wake up  
}  


void LoraInitialization(){
  /* LMIC init */
    os_init();
    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();

    /****************** ABP Only uses this section *****************************************/
    /* Set static session parameters. Instead of dynamically establishing a session
       by joining the network, precomputed session parameters are be provided.*/
    //#ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again. */
    //uint8_t appskey[sizeof(APPSKEY)];
    //uint8_t nwkskey[sizeof(NWKSKEY)];
    //memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    //memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    //LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    //#else
    /* If not running an AVR with PROGMEM, just use the arrays directly */
    //LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    //#endif
    /******************* ABP Only Ends ****************************************************/

    /* Disable link check validation */
    LMIC_setLinkCheckMode(0);

    /* Set data rate and transmit power (note: txpow seems to be ignored by the library) */
    //LMIC_setDrTxpow(DR_SF7,14); 
    LMIC_setDrTxpow(DR_SF12,20);  //lowest Datarate possible in 915MHz region
    
    /* Start job */
    //do_send(&sendjob);
}


