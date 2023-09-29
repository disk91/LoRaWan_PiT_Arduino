/* ---
 *  For LoRa Radio Node
 *  Arduino Pro or Pro Mini 
 *  328P 3.3V 8MHz
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SoftwareSerial.h>
#include <LowPower.h>
#include <SPI.h>
#include "keys.h"

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 5, LMIC_UNUSED_PIN},
};

#define TXPERIOD  (10*60) // 1 minutes
#define MODE_LINKY  0   // for linky decoding
#define MODE_OTHER  1   // for other counter like landis


void os_getArtEui (u1_t* buf) { 
  for ( int i = 0 ; i < 8 ; i++ ) buf[7-i] = APPEUI[i];
}
void os_getDevEui (u1_t* buf) { 
  for ( int i = 0 ; i < 8 ; i++ ) buf[7-i] = DEVEUI[i];
}
void os_getDevKey (u1_t* buf) {   
  memcpy_P(buf, APPKEY, 16);
}
SoftwareSerial pitinfo(4,6);

//#define DEBUG
#ifdef DEBUG
  #define LOGLN(x)  Serial.println x
  #define LOG(x) Serial.print x
  #define LOGINIT(x) Serial.begin x
  #define LOGFLUSH(x) Serial.flush x
#else
  #define LOGLN(x) 
  #define LOG(x)
  #define LOGINIT(x)
  #define LOGFLUSH(x) 
#endif

void setup() {
  // put your setup code here, to run once:
  LOGINIT((9600));
  #ifdef DEBUG
   while (!Serial);
  #endif
  LOGLN((F("GO !")));
 
  os_init();    // Intialisation de la bibliothèque
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); // agrandit la fenetre de reception
  LMIC_setAdrMode(0);  // desactive le mode ADR
  // Configure les canaux utilisables pour les communications  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);     
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);     
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);     
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);    
  // Configure la vitesse et la puissance de transmission   
  LMIC.dn2Dr = SF9; 
  LMIC_setDrTxpow(DR_SF9,8); 
  LMIC_setLinkCheckMode(0); 

  pinMode(3, OUTPUT);
  digitalWrite(3,LOW); 
}

// copy a int number into dst string
bool extractNumber(const char * src, char *dst, int maxSz) {
    for ( int idx = 0 ; idx < maxSz ; idx ++ ) {
      #if MODE_LINKY == 1
       if ( src[idx] != ',' && src[idx] != ' ' && src[idx] != '\0' ) {
      #endif
         if ( src[idx] >= '0' && src[idx] <= '9' ) { 
          dst[idx] = src[idx];
      #if MODE_LINKY == 1
         } else return false;
      #endif
       } else {
         dst[idx] = '\0';  
         return true;
       }
    }
    return false;
}

// compare str with a ref string and return true when
// str starts with ref. Case sensitive. ref can contain
// a joker char *
bool startsWith(const char * str, const char * ref) {
  if ( strlen(str) >= strlen(ref) ) {
    // possible 
    int i;
    for ( i = 0 ; i < strlen(ref) ; i++ ) {
        if ( ref[i] != '*' && str[i] != ref[i] ) {
                break;
        }
    }
    return ( i == strlen(ref) );
  }
  return false;
}

uint32_t atoi32(char * num) {
  uint32_t v = 0;
  while ( *num != '\0' && *num >= '0' && *num <= '9' ) {
    v *=10;
    v += *num - '0';
    num++;
  }
  return v;
}

// Add in hal.cpp
// uint64_t hal_compensate_tics = 0;
// u4_t hal_ticks () {
// ...
//    return (scaled | ((uint32_t)overflow << 24)) + (hal_compensate_tics);

extern uint64_t hal_compensate_tics;
void updateHalTime(uint64_t ms) {
  // We have 62.5 tics per ms -- 1 tics = 16uS
  hal_compensate_tics += (625*ms)/10;
}

void soft_reset() {
  asm volatile("jmp 0x00");
}

#define LINEBUFF_SZ 24
#define NUMBUFF_SZ  10
boolean canSleep = true;
void loop() {
  static char lbuf[LINEBUFF_SZ];
  static uint8_t ibuf = 0;
  static uint32_t lastBase = 0; 
  static uint32_t tempsMs = 0;
  static uint16_t temps = TXPERIOD-10;
  static uint16_t sleepWatchdog = 0;    // time pass with canSleep = false

/* debug logs
  Serial.println("1");
  pitinfo.begin(1200);
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);
  while ( true ) {
    if (pitinfo.available() > 0) {
      char c = pitinfo.read();
        c &= 0x7F;
        Serial.print(c);
    }
  }
*/

  uint32_t start = millis();
  if ( temps >= TXPERIOD ) {
    temps = 0;

    // wake up the PIT interface
    pinMode(3,OUTPUT);
    digitalWrite(3,HIGH);
    delay(100);
    pitinfo.begin(1200);
    // try to flush read cache
    // clean pending data, usually random values
    delay(100);
    for ( int i = 0 ; i < 50 && pitinfo.available() > 0 ; i++ ) {
      pitinfo.read();
    }
   
    // read the information on PIT during 3 seconds until we get the values
    // we are looking at and we decided to wait for going to sleep. 
    while ( (millis()-start) < 3000 && (millis() >= start) && canSleep ) {
      if ( pitinfo.available() > 0 ) {
        char c = pitinfo.read();
        c &= 0x7F;
        //Serial.print(c);
        if ( c == '\r' || c == '\n' ) {
            // end of line
           #if MODE_LINKY == 1
            if ( ibuf > 5 && startsWith(lbuf,"BASE ") ) {
              char num[NUMBUFF_SZ];
              if ( extractNumber(&lbuf[5], num, NUMBUFF_SZ) ) {
           #endif
           #if MODE_OTHER == 1
            if ( ibuf > 6 && startsWith(lbuf,"EAP_s ") ) {
              char num[NUMBUFF_SZ];
              if ( extractNumber(&lbuf[6], num, NUMBUFF_SZ) ) {
           #endif
                uint32_t base = atoi32(num);
                if ( lastBase == 0 || base >= lastBase ) {
                  num[0] = (base >> 24) & 0xFF;
                  num[1] = (base >> 16) & 0xFF;
                  num[2] = (base >>  8) & 0xFF;
                  num[3] = (base      ) & 0xFF;
                  lastBase = (lastBase==0)?0:( base - lastBase ); // save 1 variable
                  num[4] = (lastBase >> 16) & 0xFF;
                  num[5] = (lastBase >>  8) & 0xFF;
                  num[6] = (lastBase      ) & 0xFF;
                  // Process
                  LOG((F("***")));
                  LOG((base));
                  LOGLN((F("***")));
                  lbuf[0] = ' '; // make sure we reload new data to enter here (seen bug due to low memory)
                  lastBase = base;
                  LMIC_setDrTxpow(DR_SF10,8); 
                  canSleep = false;
                  lmic_tx_error_t err = LMIC_setTxData2(1, num, 7, 0);
                  if ( err != 0 ) {
                    canSleep=true;
                    LOG((F("Tx Err")));
                    LOGLN((err));
                  }
                  break;
                } else {
                  LOG((F("Erreur lecture")));
                }
                
              }
            }
        } else {
           if ( c >= ' ' && c <= 'z' && ibuf < LINEBUFF_SZ ) {
            lbuf[ibuf] = c;
            ibuf++;
           } else {
            // on a 3 et 2 en debut de bloc de transmission
            //Serial.print("**");
            //Serial.print((int)c);
           }
        }
      }
    }
    ibuf = 0;
    pitinfo.end();
    digitalWrite(3,LOW);
  } else {
    delay(1);
  }
  
  os_runloop_once();

  // update the running time for next message schedule
  uint32_t stop = millis();
  if ( stop > start ) tempsMs += (stop - start);
  while ( tempsMs > 1000 ) {
    temps++;        // count seconds
    tempsMs -= 1000;

    // manage LoRaWan watchdog
    if ( ! canSleep ) {
       sleepWatchdog++;
    }
  }
  if ( canSleep ) {
    sleepWatchdog = 0;
    LOGFLUSH(());
    LowPower.powerDown(SLEEP_8S, ADC_OFF,BOD_OFF);
    LOGINIT((9600));
    LOG(("."));
    temps += 8;
    updateHalTime(8000);
  } else {
    if ( sleepWatchdog > 120 ) {
       soft_reset();
    }
  }
  LOG(("tics / ms : "));LOG((hal_ticks()));LOGLN(((10*hal_ticks())/625));
}

void onEvent (ev_t ev) {
    LOG(("Ev : "));
    LOGLN((ev));
    switch(ev) {
        case EV_JOINED:
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_TXCOMPLETE:
        case EV_JOIN_FAILED:
        case EV_TXCOMPLETE: 
        case EV_TXCANCELED:
        case EV_SCAN_TIMEOUT:
        case EV_BEACON_FOUND:
        case EV_BEACON_MISSED:
        case EV_BEACON_TRACKED:
        case EV_REJOIN_FAILED:
        case EV_RFU1:
        case EV_LOST_TSYNC:
        case EV_RESET:
        case EV_RXCOMPLETE:
        case EV_LINK_DEAD:
            canSleep = true;
            break;
        case EV_TXSTART:
        case EV_JOINING:
        case EV_RXSTART:
            canSleep = false;
        default:
            break;
    }
}
