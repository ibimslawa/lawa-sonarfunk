/* Lars Warich Sonar+Funk R1 (Revision 1)
 * DMX Taster-Fernbedienung.
 * GEMEINSAME BIBLIOTHEK
 * Ver 25-03-14 */

#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>

using namespace std;

// #define DEBUG

#pragma region "DEFINES & CONSTANTS"

/* Highest GPIO pin number on ESP32 Dev4 boards. */
#define ESP_DEV4_GPIO_MAX 33

/* Max Anzahl der gesendeten Funktions-Taster */
#define FUNBTN_COUNT 4

/* Senderate fuer Alive Nachrichten. */
#define TICK_ALIVE 1000
/* Zeitdauer 1Hz. */
#define TICK_1HZ 1000
/* Zeitdauer 2Hz. */
#define TICK_2HZ 500
/* Zeitdauer 4Hz. */
#define TICK_4HZ 250
/* Zeitdauer fuer LED Flash. */
#define TICK_LED_FLASH 120
/* Zeitdauer bis Taster als gehalten gilt. */
#define DUR_BTN_HOLD 400
/* Zeitdauer fuer Taster Hysterese (Entprellen) */
#define DUR_BTN_HYST 160

/* PWM halbe Helligkeit Duty Cycle */
enum PWM_GAMMA22_8B : uint8_t {
  PWM_DIM = 0x32, /* => 42%  */
  PWM_BRT = 0x8a, /* => 75%  */
  PWM_FULL = 0xff /* => 100% */
};

/* string length for MAC address. */
#define MAC_STR_LEN 18
/* An alle Peer-Teilnehmer senden. */
#define MAC_PEER_ALL nullptr
/* MAC Addresse des Empfaenger. */
#define MAC_ADR_RCVR__M {0x4c, 0x57, 0x2a, 0x73, 0x66, 0x45}
/* MAC Adresse des Senders. */
#define MAC_ADR_XTMR__M {0x4c, 0x57, 0x2a, 0x73, 0x66, 0x53}
/* MAC OUI fuer Sonar+Funk "LW*". */
static const uint8_t MACOUI_SF[3] = {0x4c, 0x57, 0x2a};
/* WiFi Paket Subtype fuer einen Action Frame */
static const uint8_t ACTION_SUBTYPE = 0xd0;

/* RSSI dBm-Wert der als "nicht verbunden" gilt. */
#define RSSI_NO_CONN -93

/* Aktion eines Funktiontasters. */
enum FUNBTN_ACTION : uint8_t {
  FUNBTN_BTNDOWN = 'D',    /* Taster-Aktion Druecken */
  FUNBTN_BTNUP = 'U',      /* Taster-Aktion Freigabe */
  FUNBTN_ACTN_REPEAT = 'R' /* Taster-Aktion Repeat (Nachricht wiederholen) */
};

/* Index im Byte fuer den jeweiligen Fehler */
enum SOFU_ERRORCODE_BIT : uint8_t {
  ERRBIT_NO_CONN = 0, /* Connection now lost */
  /* Connection was lost (under ) */
  ERRBIT_WIFI = 2,      /* Problem with WiFi initialisation */
  ERRBIT_ENOW = 3,      /* Problem with ESP-NOW initialisation or sending */
  ERRBIT_DMX = 4 ,       /* Problem with DMX initialisation or sending */
  ERRBIT_MIDI = 5       /* Problem with MIDI initialisation or sending */
};

#pragma endregion

#pragma region "TYPE DEFINITIONS"

struct wifi_ieee80211_mac_hdr_t {
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
};

struct wifi_ieee80211_packet_t {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
};

typedef uint32_t stamp_ui32_t;

#define SOFU_PKT_KEY__M {'S', '+', 'F'}
#define SOFU_PKT_KEY_END 3
#define STR_F_PKT_TASTER "%c / %hhu / %u / 0b%4s / %1hhx / %c"
enum SOFU_PKT_TYPE : uint8_t {
  PKT_TYPE_ALIVE = 'A',
  PKT_TYPE_FUNBTN = 'B',
  PKT_TYPE_POTI = 'P'
};
struct sofu_pkt_primiv_t {
public:
  const char key[3] = SOFU_PKT_KEY__M;
  char type = '#';
  uint8_t len = sizeof(sofu_pkt_primiv_t);
  stamp_ui32_t stamp = 0;

  /* Prueft ob ein Byte Array ein valides SOFU Paket ist. */
  static bool valides_pkt(const uint8_t *in_data, int len) {
    return (len > SOFU_PKT_KEY_END) && (in_data[0] == 'S' && in_data[1] == '+' && in_data[2] == 'F');
  }
};
struct sofu_pkt_alive_t : sofu_pkt_primiv_t {
public:
  bool alive_ack = false;
  sofu_pkt_alive_t(stamp_ui32_t _stamp = 0, bool _alive_ack = false) {
    type = PKT_TYPE_ALIVE;
    len = sizeof(sofu_pkt_alive_t);
    stamp = _stamp;
  };
};
struct sofu_pkt_funbtn_t : sofu_pkt_primiv_t {
public:
  uint8_t btn_byte;
  uint8_t lastFunbtnId;
  FUNBTN_ACTION lastActn;
  sofu_pkt_funbtn_t() {
    type = PKT_TYPE_FUNBTN;
    len = sizeof(sofu_pkt_funbtn_t);
  }
  sofu_pkt_funbtn_t(stamp_ui32_t stamp, uint8_t btn_byte, uint8_t lastFunbtnId, FUNBTN_ACTION lastActn)
      : sofu_pkt_funbtn_t() {
    this->stamp = stamp;
    this->btn_byte = btn_byte;
    this->lastFunbtnId = lastFunbtnId;
    this->lastActn = lastActn;
  };
  String toString() {
    char strc[35];
    sprintf(strc, "FBTN St:%04u By:%1hhx Id:%2hhx A:%c", stamp, btn_byte, lastFunbtnId, (char)lastActn);
    String str = (String)strc;
    return str;
  }
};

/* Typklasse fuer die Darstellung eines RSSI in dBm und Prozent.
 * Verwendet Wert in dBm, wenn verwendet wie eine Variable. Member perc gibt Wert in Prozent an. */
template <typename T_rssi> class RssiValue {
public:
  /* Definiert ein RssiValue Objekt mit unterer und oberer Grenze.
   * @param p0 Untere dBm-Grenze fuer 0%
   * @param p100 Obere dBm-Grenze fuer 100% */
  RssiValue(const signed int f, T_rssi *proz_p, const signed int p0, const signed int p100) {
    _dbm = f;
    updtVal(f);
    _p0 = p0;
    _p100 = p100;
    proz = proz_p;
  }
  virtual ~RssiValue() {
  }
  virtual signed int &operator=(const signed int &f) {
    updtVal(f);
    return _dbm = f;
  }
  virtual const signed int &operator()() const {
    return _dbm;
  }
  virtual bool operator>(const signed int &r) {
    return r < _dbm;
  }
  virtual bool operator<=(const signed int &r) {
    return !(r < _dbm);
  }
  virtual bool operator>=(const signed int &r) {
    return !(_dbm < r);
  }
  virtual operator String() const {
    return (String)_dbm + "dBm";
  }
  String prozStr() const {
    return (String)*proz + "%";
  }
  /* Darstellung des RSSI in Prozent (Min dBm = 0%, Max dBm = 100%). */
private:
  signed int _dbm;
  int _p0 = -93;
  int _p100 = -50;
  T_rssi *proz;
  void updtVal(signed int f) {
    if ( proz != nullptr ) {
      if ( f <= _p0 )
        *proz = 0;
      else if ( f >= _p100 )
        *proz = 100;
      else
        *proz = map(f, _p0 + 1, _p100, 1, 100);
    }
  }
};

#pragma endregion

#pragma region "HELPER FUNCTIONS"

int lenof(byte *arr);
int lenof(const byte *arr);
int lenof(int *arr);
int lenof(uint16_t *arr);

template <typename T> const uint8_t *asbytesptr(T data_p) {
  return reinterpret_cast<const uint8_t *>(data_p);
}

/* Bit aus Byte zu char auslesen.
 * @param by Byte
 * @param pos Pos 0-basiert
 * @return char '1' oder '0' */
char bittochr(unsigned char by, unsigned short pos);

/* Output des Pins wird gewechselt. */
void digitalToggle(uint8_t pin);

/* Konvertiert eine MAC Adresse von Byte Array zu String. */
String macAddrToStr(const uint8_t *mac);

/* Gibt die MAC Adresse des ESP32 als String zurueck. */
String getMacAddrStr_esp();

/* Sendet eine ESP-NOW Nachricht an spezifische MAC. */
void EspNowSendMsg_SoFu(const uint8_t peer_mac[6], const uint8_t *data, size_t data_len);

/* Standard Callback NOW Paket empfangen. */
void OnNowPktRecv_SoFu(const uint8_t *nowpkg, int len);