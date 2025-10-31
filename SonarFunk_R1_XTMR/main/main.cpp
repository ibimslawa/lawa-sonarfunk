/* Lars Warich Sonar+Funk R1 (Revision 1)
 * DMX Wireless Button Remote
 * TRANSMITTER MODULE
 * Ver 25-07-22 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "lawa_sonarfunk_lib.h"
#include <Preferences.h>

#define FIRMWARE_VER (String) "25-07d"
/* Typ des ESP32 Boards. Define aendern fuer anderes Board. */
#define BOARD_ESP32C3SMINI

// #define DEBUG

#pragma endregion

#pragma region "TRANSMITTER DEFINES & CONSTANTS"

#define BUTTON_COUNT 4
const uint8_t MAC_ME[] = MAC_ADR_XTMR__M;
const uint8_t MAC_PEER[] = MAC_ADR_RCVR__M;

#define PREFS "prefs"
/* Preferences key: Button order */
#define PREFS_BTNS_SWAPPED "btnorder"

// #define SEND_TWICE

#pragma endregion

#pragma region "PIN DEFINES"

#ifdef BOARD_ESP32DEV
#define OUT_LED_ACTY 2  /* Aktivitaet LED_BUILTIN */
#define OUT_LED_ERROR 4 /* Fehler */
#define PINS_FKTTSTR {12, 14, 26, 27};
#define LED_BOARD 8
#endif
#ifdef BOARD_ESP32C3SMINI
#define OUT_LED_ACTY 20  /* Aktivitaet */
#define OUT_LED_ERROR 21 /* Fehler */
#define PINS_FKTTSTR {10, 8, 7, 6}
#endif

#define PWMFREQ_LED 10000
#define PWMRES_LED 10
#define PWMCH_LED_ACTY 0
#define PWMCH_LED_FEHLER 1

#pragma endregion

#pragma region "MORE GLOBAL VARIABLES"

std::array<uint8_t, FUNBTN_COUNT> IN_FUNBTNS = PINS_FKTTSTR;
/* Bitweiser Fehlercode */
static uint8_t errorCode = 0b0;
/* Nachrichten-Stempel Sender. */
static stamp_ui32_t sndr_stamp = 1;
/* Speichert ob ein Taster betatigt ist. */
static uint8_t funbuttonByte = 0b0000;
/* Verwaltung der Flash Einstellungen */
static Preferences preferences;

/* ZEITSTEMPEL */

static unsigned long ts_tick2hz = 0;
static unsigned long ts_activity = 0;
static unsigned long ts_lastBtnEvent = 0;

#pragma endregion

#pragma region "DECLARATIONS"

void sendFunbtnPkt(uint8_t byteFTstr, uint8_t bitFTstr_neu, FUNBTN_ACTION aktion);
void OnFunbtnDown(uint8_t byteFTstr, uint8_t bitFTstr_neu);
void OnFunbtnUp(uint8_t byteFTstr, uint8_t bitFTstr_neu);
void OnDataRecv_Xmtr(const uint8_t *mac, const uint8_t *in_data, int len);

#pragma endregion

#pragma region "INITIALISATION FUNCTIONS"

void initEspNow_Sndr() {
  Serial.println("[INFO] Initialisiere ESP-NOW...");
  if ( esp_now_init() != ESP_OK ) {
    Serial.println("[ERROR] Fehler bei der Initialisierung von ESP-NOW");
    bitSet(errorCode, ERRBIT_ENOW);
  } else {
    /* Peer Partner hinzufuegen: */
    esp_now_peer_info_t peerInfo = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
    memcpy(peerInfo.peer_addr, MAC_PEER, 6);
    if ( esp_now_add_peer(&peerInfo) != ESP_OK ) {
      Serial.println("[ERROR] Fehler beim Hinzufuegen eines Peers");
      bitSet(errorCode, ERRBIT_ENOW);
    }
    /* Register callback function for ESP-NOW receiver: */
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv_Xmtr));
  }
}

void blinkLedc(uint8_t ch, uint8_t blinks, uint16_t interval = 100) {
  for ( ; blinks > 0; --blinks ) {
    if ( ledcRead(ch) > 0x0 ) ledcWrite(ch, 0x0);
    delay(interval);
    ledcWrite(ch, PWM_FULL);
    delay(interval);
  }
  ledcWrite(ch, 0x0);
}

#pragma endregion

#pragma region "MAIN FUCNTIONS"

void setup() {
  Serial.begin(115200);
  Serial.println("\nLAWA SONAR+FUNK R1 - TRANSMITTER starting");
  Serial.println("Firmware Ver " + FIRMWARE_VER + "\n");

  /* PWM initialisieren */
  ledcSetup(PWMCH_LED_ACTY, PWMFREQ_LED, PWMRES_LED);
  ledcSetup(PWMCH_LED_FEHLER, PWMFREQ_LED, PWMRES_LED);
  /* Pin Modes setzen: */
  ledcAttachPin(OUT_LED_ACTY, PWMCH_LED_ACTY);
  ledcAttachPin(OUT_LED_ERROR, PWMCH_LED_FEHLER);
  for ( uint8_t pin : IN_FUNBTNS )
    pinMode(pin, INPUT_PULLDOWN);

  /* LED Test Start:*/
  ledcWrite(PWMCH_LED_ACTY, PWM_BRT);
  ledcWrite(PWMCH_LED_FEHLER, PWM_BRT);

  /* WiFi initialisieren: */
  Serial.println("[INFO] Initialisiere WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  Serial.println("[INFO] Fehler 0x300a ignorieren.");
  // Override MAC addressss
  esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, &MAC_ME[0]);
  if ( err == ESP_OK ) {
    Serial.println("[ OK ] Erfolgreich MAC geaendert. MAC: " + getMacAddrStr_esp());
  }

  /* ESP-NOW initialisieren: */
  initEspNow_Sndr();

  /* LED Test Fortfuehrung: */
  delay(1500);
  ledcWrite(PWMCH_LED_ACTY, 0x0);
  ledcWrite(PWMCH_LED_FEHLER, 0x0);

  /* Einstellungen laden */
  bool reihenfolgeTaster = false;
  if ( preferences.begin(PREFS, false) ) {
    reihenfolgeTaster = preferences.getBool(PREFS_BTNS_SWAPPED, false);
    if ( digitalRead(IN_FUNBTNS[0]) == HIGH ^ digitalRead(IN_FUNBTNS[FUNBTN_COUNT - 1]) == HIGH ) {
      reihenfolgeTaster = (digitalRead(IN_FUNBTNS[FUNBTN_COUNT - 1]) == HIGH);
      preferences.putBool(PREFS_BTNS_SWAPPED, reihenfolgeTaster);
      Serial.println("[ OK ] Einstellung: Taster Reihenfolge\n");
    }

    preferences.end();
  }
  /* Switch order of buttons */
  if ( reihenfolgeTaster ) {
    std::array<uint8_t, FUNBTN_COUNT> temp = PINS_FKTTSTR;
    for ( short i = 0; i < FUNBTN_COUNT; i++ )
      IN_FUNBTNS[FUNBTN_COUNT - 1 - i] = temp[i];
  };
  delay(500);
  blinkLedc(PWMCH_LED_ACTY, (reihenfolgeTaster ? 2 : 1), 250);

  Serial.println("[INFO] Setup fertig! \n");
  delay(1000);

  for ( uint8_t b = 0; b < BUTTON_COUNT; b++ )
    if ( digitalRead(IN_FUNBTNS[b]) ) bitSet(funbuttonByte, b);
} /* setup */

void loop() {
  /* Taster abfragen: */
  bool bttg = false;
  if ( millis() - ts_lastBtnEvent >= DUR_BTN_HYST ) {
    for ( uint8_t b = 0; b < BUTTON_COUNT; b++ ) {
      bttg = bitRead(funbuttonByte, b);
      if ( !bttg && digitalRead(IN_FUNBTNS[b]) == HIGH ) {
        /* Pos Flanke = Taster Betatigung, entprellt um DUR_BTN_HYST */
        bitSet(funbuttonByte, b);
        OnFunbtnDown(funbuttonByte, b);
        ts_lastBtnEvent = millis();
      } else if ( bttg && digitalRead(IN_FUNBTNS[b]) == LOW ) {
        /* Neg Flanke = Taster Freigabe, entprellt um DUR_BTN_HYST */
        bitClear(funbuttonByte, b);
        OnFunbtnUp(funbuttonByte, b);
        ts_lastBtnEvent = millis();
      }
    }
  }

  /* Aktivitaet Status LED: */
  if ( millis() - ts_activity >= TICK_LED_FLASH ) {
    ledcWrite(PWMCH_LED_ACTY, PWM_DIM);
  }

  /* Fehler LED: */
  if ( bitRead(errorCode, ERRBIT_NO_CONN) ) {
    if ( millis() - ts_tick2hz >= TICK_2HZ ) {
      ts_tick2hz = millis();
      ledcWrite(PWMCH_LED_FEHLER, (ledcRead(PWMCH_LED_FEHLER) < PWM_BRT) ? PWM_BRT : 0);
    }
  } else
    ledcWrite(PWMCH_LED_FEHLER, (errorCode > 0) ? PWM_BRT : 0);

  /* Connection lost. (Only triggered once lost): */
  if ( !bitRead(errorCode, ERRBIT_NO_CONN) && millis() - ts_activity >= 3 * TICK_ALIVE ) {
    bitSet(errorCode, ERRBIT_NO_CONN);
    Serial.println("[ERROR] Connection to receiver has been lost!");
  }

} /* loop */

/* Status der Funktionstaster senden. Nachricht ueber ESP-Now. */
void sendFunbtnPkt(uint8_t byteFTstr, uint8_t bitFTstr_neu, FUNBTN_ACTION aktion) {
  /* Werte Funktionstaster doppelt senden: */
  sofu_pkt_funbtn_t pkt = {sndr_stamp, byteFTstr, bitFTstr_neu, aktion};
  EspNowSendMsg_SoFu(MAC_PEER, asbytesptr(&pkt), pkt.len);
#ifdef SEND_TWICE
  EspNowSendMsg_SoFu(MAC_PEER, asbytesptr(&pkt), pkt.len);
#endif
  /* Stempel erhoehen wenn aktion kein Repeat: */
  if ( aktion != FUNBTN_ACTN_REPEAT ) sndr_stamp++;
}

#pragma endregion

#pragma region "CALLBACKS"

/* Callback Taster Druecken. */
void OnFunbtnDown(uint8_t byteFTstr, uint8_t bitFTstr_neu) {
  ts_activity = millis();
  ledcWrite(PWMCH_LED_ACTY, 0);
  sendFunbtnPkt(byteFTstr, bitFTstr_neu, FUNBTN_BTNDOWN);
#ifdef DEBUG
  Serial.printf("T:%2d ", byteFTstr);
#endif
}

/* Callback Taster Freigabe. */
void OnFunbtnUp(uint8_t byteFTstr, uint8_t bitFTstr_neu) {
  sendFunbtnPkt(byteFTstr, bitFTstr_neu, FUNBTN_BTNUP);
}

/* Callback Empfangenes WiFi Paket verarbeiten. */
void OnDataRecv_Xmtr(const uint8_t *mac, const uint8_t *in_data, int len) {
  if ( !sofu_pkt_primiv_t::valides_pkt(in_data, len) ) {
    Serial.println("[INFO] ESP-NOW Unrecognisable packet.");
    return;
  }
  if ( in_data[SOFU_PKT_KEY_END] == PKT_TYPE_ALIVE ) {
    /* Alive empfangen, Acknowledgement antworten: */
    sofu_pkt_alive_t pkt;
    pkt.alive_ack = true;
    EspNowSendMsg_SoFu(MAC_PEER, asbytesptr(&pkt), pkt.len);
  }
  /* LED Aktivitaet blinken: */
  ts_activity = millis();
  ledcWrite(PWMCH_LED_ACTY, PWM_BRT);
  /* Verbindung wiederhergestellt: */
  if ( bitRead(errorCode, ERRBIT_NO_CONN) ) {
    sendFunbtnPkt(funbuttonByte, UINT8_MAX, FUNBTN_ACTN_REPEAT);
    Serial.println("[INFO] Conneciton to receiver reestablished.");
  }
  bitClear(errorCode, ERRBIT_NO_CONN);
}

#pragma endregion
