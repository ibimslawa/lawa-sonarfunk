#include "lawa_sonarfunk_lib.h"

int lenof(byte *arr) {
  return sizeof(arr) / sizeof(arr[0]);
}
int lenof(const byte *arr) {
  return sizeof(arr) / sizeof(arr[0]);
}
int lenof(int *arr) {
  return sizeof(arr) / sizeof(arr[0]);
}
int lenof(uint16_t *arr) {
  return sizeof(arr) / sizeof(arr[0]);
}

char bittochr(unsigned char by, unsigned short pos) {
  return ((by >> pos) & 0x1) == 1 ? '1' : '0';
}

void digitalToggle(uint8_t pin) {
  digitalWrite(pin, (digitalRead(pin) == HIGH) ? LOW : HIGH);
}

String macAddrToStr(const uint8_t *mac) {
  char buffer[MAC_STR_LEN];
  sprintf(buffer, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return (String)buffer;
}

String getMacAddrStr_esp() {
  uint8_t espMac[6];
  esp_err_t ec = esp_wifi_get_mac(WIFI_IF_STA, espMac);
  if ( ec == ESP_OK ) {
    return macAddrToStr(espMac);
  } else {
    return (String) "??:??:??:??:??:??";
  }
}

void EspNowSendMsg_SoFu(const uint8_t peer_mac[6], const uint8_t *data, size_t data_len) {
  esp_err_t result = esp_now_send(peer_mac, data, data_len);
  if ( result != ESP_OK ) {
    Serial.println("[ERROR] Sending packert failed.");
    for ( size_t i = 0; i < data_len; i++ )
      Serial.printf("%hhx ", data[i]);
    Serial.println();
  }
}


