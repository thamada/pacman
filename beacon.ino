//Time-stamp: <2024-06-17 11:52:30 hamada>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <LowPower.h>

//#define SLEEP_S 180 // Sleep time. Set in units of 10 seconds.
#define SLEEP_S 5 // Sleep time. Set in units of 10 seconds.

/** GPS001
    static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
    static const u1_t PROGMEM DEVEUI[8] = { 0xE4, 0x55, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
    static const u1_t PROGMEM APPKEY[16] = { 0x48, 0x3A, 0x8A, 0x28, 0x76, 0xC5, 0x46, 0x31, 0x5D, 0x45, 0x9C, 0x06, 0x22, 0x8B, 0x43, 0x23 };// App Key: msbで記述
**/

/** GPS007
    static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
    static const u1_t PROGMEM DEVEUI[8] = { 0x9E, 0x55, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
    static const u1_t PROGMEM APPKEY[16] = { 0x10, 0x56, 0xEE, 0xB8, 0x4D, 0x6E, 0x3A, 0x90, 0x87, 0x69, 0x68, 0xDB, 0x90, 0x9D, 0x00, 0x54 };// App Key: msbで記述
**/

/** GPS008
    static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
    static const u1_t PROGMEM DEVEUI[8] = { 0x4A, 0x57, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
    static const u1_t PROGMEM APPKEY[16] = { 0x06, 0xA5, 0x0A, 0x03, 0xDD, 0x50, 0xEB, 0x61, 0xB2, 0x83, 0xF8, 0xCE, 0x56, 0xCD, 0xC1, 0x6C };// App Key: msbで記述
**/

/** GPS009
    static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
    static const u1_t PROGMEM DEVEUI[8] = { 0x50, 0x57, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
    static const u1_t PROGMEM APPKEY[16] = { 0x0B, 0x10, 0x37, 0xF8, 0x6B, 0xBD, 0xD1, 0xBE, 0xA6, 0xFA, 0x54, 0x8D, 0xC8, 0x13, 0xFA, 0x46 };// App Key: msbで記述
**/

/** GPS006
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0xDE, 0x57, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0x07, 0x4B, 0x5D, 0x09, 0x6F, 0xBD, 0x98, 0xE2, 0xC8, 0x76, 0xA4, 0xAA, 0xB2, 0x03, 0xBC, 0x68 };// App Key: msbで記述
**/

/** beacon-006
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0xEE, 0x61, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0x41, 0x6A, 0x5B, 0x1A, 0x0A, 0x0E, 0xDB, 0x96, 0x16, 0xA7, 0x30, 0xAF, 0xC5, 0x94, 0x25, 0x07 };// App Key: msbで
 **/

/** beacon-001 
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0x0F, 0x62, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0x44, 0x4F, 0x94, 0xE3, 0xF6, 0xEB, 0x6C, 0x64, 0xD7, 0x46, 0x8B, 0x19, 0x39, 0x8A, 0x6F, 0x1C };// App Key: msbで
**/

/** beacon-008 
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0xE5, 0x62, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0x8B, 0xF1, 0x84, 0x8E, 0x2F, 0x22, 0xD7, 0xA2, 0x7B, 0x42, 0x50, 0x11, 0x1F, 0x27, 0x12, 0xDB };// App Key: msbで
**/

/** beacon-007 
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0x3A, 0x63, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0xDC, 0xCE, 0x43, 0x03, 0xC2, 0xC1, 0x58, 0x85, 0xC1, 0xEE, 0xE0, 0x64, 0x8A, 0xF3, 0xA8 };// App Key: msbで
**/

/** beacon-004
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0x08, 0x66, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0xF3, 0xE3, 0x3F, 0xE9, 0x1F, 0x03, 0xC5, 0xA1, 0x01, 0x62, 0xE6, 0x34, 0xB1, 0xE0, 0xE8, 0x32 };// App Key: msbで
**/

/** Dummy 
static const u1_t PROGMEM APPEUI[8] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 };// App Key: msbで
**/

/** pacman **/
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App EUI: lsbで記述 0000000000000000でも構わない
static const u1_t PROGMEM DEVEUI[8] = { 0x82, 0x86, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Device EUI: lsbで記述
static const u1_t PROGMEM APPKEY[16] = { 0xC3, 0x25, 0x11, 0xEE, 0xC3, 0x78, 0x5E, 0x95, 0x1A, 0x20, 0xE1, 0x7F, 0xAD, 0xAD, 0x48, 0x8C };// App Key: msbで



void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 0;


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

//-- AHR-L01ボード PIN割り当て----
// Board Ver.2.0
#define VCC_ENABLE 5 // D5 Power(+) 
#define LED A2    // A2 LED
// A4 SDA/SDI white cable
// A5 SCL/SCK yellow cable
//------------------------------
// --Cayenne用設定 ---------------
CayenneLPP lpp(51);                // create a buffer of 51 bytes to store the payload
//------------------------------


// センサデータの取得
void get_data() {
  static uint8_t counter = 0;

  /**
  float  mydata[9];
  mydata[0] = cpuTemp();
  mydata[1] = cpuVcc();
  Serial.flush();
  ledBrink(3, 100);
  **/

  // CayenneLPP
  lpp.reset();                              // clear the buffer
  //lpp.addAnalogOutput(1, mydata[0]); // AnalogOutput: 4-bytes
  lpp.addDigitalOutput(1, counter++);  // DigitalOutput: 3-bytes !!
}

void do_send(osjob_t* j) {

  delay(1000);

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    Serial.println(F("LMIC SET DN"));
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
    //Serial.println(LMIC.freq);
  }
  // Next TX is scheduled after TX_COMPLETE event.

  digitalWrite(A2, LOW);

}
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
    //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

    /**
   for (int i = 0; i < SLEEP_S / 10 ; i++) {

      // Use library from https://github.com/rocketscream/Low-Power
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    }
    **/
    Serial.println(F("sleeping..."));
    Serial.flush();
    delay(SLEEP_S * 1000);
    Serial.println(F("...!"));
    Serial.flush();

    Serial.println(F("Get data and do_send command"));
    get_data();
    do_send(&sendjob);
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

void setup() {
  Serial.begin(9600);
  while (!Serial);
  //  Serial.println(F("beacon-004"));
  Serial.println(F("Dummy"));
  Serial.println(F("fw: 20240617 11:52"));
  Serial.print(F("SLEEP_S = "));
  Serial.println(SLEEP_S);
  Serial.println(F("only Beacon!"));


  delay(5000);

  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  pinMode(LED, OUTPUT);
  ledBrink(6, 200);
  delay(2000);
  ledBrink(6, 500);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  /*
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // TTN uses SF9 for its RX2 window.
    //LMIC.dn2Dr = AS923_DR_SF9;
    LMIC.dn2Dr = DR_SF9;
    Serial.println(F("LMIC SET DN"));
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(AS923_DR_SF7,14);
    //LMIC_setDrTxpow(AS923_DR_SF10,13);
    LMIC_setDrTxpow(DR_SF10,13); //20mW=13dBm
    Serial.println(F("LMIC SET TX"));
  */

  get_data();
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

float cpuTemp() {                    // CPU温度測定関数
  long sum = 0;
  adcSetup(0xC8);                    // Vref=1.1V, input=ch8
  for (int n = 0; n < 100; n++) {
    sum = sum + adc();               // adcの値を読んで積分
  }
  return (sum * 1.1 / 102.4) - 342.5; // 温度を計算して戻り値にする。-342.5は要調整
}

float cpuVcc() {                     // 電源電圧(AVCC)測定関数
  long sum = 0;
  adcSetup(0x4E);                    // Vref=AVcc, input=internal1.1V
  for (int n = 0; n < 10; n++) {
    sum = sum + adc();               // adcの値を読んで積分
  }
  return (1.1 * 10240.0) / sum;      // 電圧を計算して戻り値にする
}

void adcSetup(byte data) {           // ADコンバーターの設定
  ADMUX = data;                      // ADC Multiplexer Select Reg.
  ADCSRA |= ( 1 << ADEN);            // ADC イネーブル
  ADCSRA |= 0x07;                    // AD変換クロック CK/128
  delay(10);                         // 安定するまで待つ
}

unsigned int adc() {                 // ADCの値を読む
  unsigned int dL, dH;
  ADCSRA |= ( 1 << ADSC);            // AD変換開始
  while (ADCSRA & ( 1 << ADSC) ) {   // 変換完了待ち
  }
  dL = ADCL;                         // LSB側読み出し
  dH = ADCH;                         // MSB側
  return dL | (dH << 8);             // 10ビットに合成した値を返す
}

// LED点滅関数関数（点滅回数、点滅周期[ms]）
void ledBrink(char blink_num, unsigned int blink_int) {
  for (char i = 0; i < (blink_num * 2); i++) {
    delay(blink_int / 2);
  }
}

/**
// LED点滅関数関数（点滅回数、点滅周期[ms]）
void ledBrink(char blink_num, unsigned int blink_int) {
  for (char i = 0; i < (blink_num * 2); i++) {
    if (digitalRead(LED) != HIGH)digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
    delay(blink_int / 2);
  }
  digitalWrite(LED, LOW);
}
*/
