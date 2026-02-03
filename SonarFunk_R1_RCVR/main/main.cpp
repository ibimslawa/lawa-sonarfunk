/* Lars Warich Sonar+Funk R1 (Revision 1)
 * DMX Wireless Button Remote
 * RECEIVER MODULE
 * Ver 25-07-22 */

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <driver/gpio.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string>

#include "esp_dmx.h"
#include "lawa_sonarfunk_lib.h"
#include "rcvr_display.h"
#include "rcvr_types.h"

#define FIRMWARE_VER (const char *)"26-01a"

// #define DEBUG
// #define DEBUG_WIREL
// #define DEBUG_DISP

#pragma region "RECEIVER DEFINES & CONSTANTS"

static const uint8_t MAC_ME[] = MAC_ADR_RCVR__M;
static const uint8_t MAC_PEER[] = MAC_ADR_XTMR__M;

/* Zeitdauer fuer Display Aktualisierung. */
#define DUR_DISP_UPDT 500
/* Zeitdauer fuer Menuename einblenden. */
#define DUR_DISP_MNAME 600
/* Zeitdauer fuer widerholte Aktionen bei Taster halten. */
#define TICK_BTN_HOLD 30
/* DMX Frequenz minimale Intervallzeit */
#define DMX_TICK_MIN 25
/* Durations for blinking in edit mode */
static const short DUR_DISP_EDIT[] = {250, 400};
/* Zeitdauer für das Anzeigen einer verlorenen Verbindung */
#define DUR_ERR_LED_CONN 60000

#define PREFS "prefs"
/* Preferences key: Personality */
#define PREFS_PERS_NUM "pers"
/* Preferences key: DMX address */
#define PREFS_DMX_ADRS "dmx_adrs"
/* Preferences key: DMX Tick */
#define PREFS_DMX_TICK "dmx_tick"
/* Preferences key: MIDI channel */
#define PREFS_MIDI_CHAN "midi_chan"
/* Preferences key: MIDI first note */
#define PREFS_MIDI_NOTE "midi_note"

#define SCROLLTEXT_MODELNAME (char *)"LAWA Sonar+Funk R1"

/* Time to impede certain actions */
static uint16_t wait_after_setup = 5000;

#pragma endregion

#pragma region "DEFINES PINS + DMX + MIDI"

enum MENUBTN_ID : uint8_t {
    MENU_DN = 0, /* Taster "runter" */
    MENU_UP = 1, /* Taster "hoch" */
    MENU_OK = 2, /* Taster "OK" */
    MENUBTN__COUNT = 3
};
struct menubtn_t {
    const unsigned short bit;
    const uint8_t pin;
};
enum MENUBTN_STATE : uint8_t {
    MENUBTN_STATE_NONE = 0,
    MENUBTN_STATE_DOWN = 1,
    MENUBTN_STATE_PRESSED = 2,
    MENUBTN_STATE_HOLD = 3
}

static const menubtn_t IN_MENUBTNS[] = {{MENU_DN, 35}, {MENU_UP, 34}, {MENU_OK, 39}};
static const gpio_config_t IN_MENUBTNS_GPIO_CFG = {
    .pin_bit_mask =
        ((1ULL << IN_MENUBTNS[MENU_DN].pin) | (1ULL << IN_MENUBTNS[MENU_UP].pin) | (1ULL << IN_MENUBTNS[MENU_OK].pin)),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_DISABLE};
#define OUT_LED_ACTY 2  /* LED Aktivity IO2=LED_BUILTIN */
#define OUT_LED_ERROR 4 /* LED Error */
#define OUT_DMX_TX 17   /* DMX TX UART2 */
#define OUT_DMX_RX 16   /* DMX RX UART2 */
#define OUT_DMX_RTS 21  /* DMX RTS UART (NC) */
#define OUT_MIDI_TX 23  /* MIDI TX UART1 */
#define OUT_MIDI_RX 22  /* MIDI RX UART1 (not used) */
static const slx2016_d4_pins_t OUT_DISP = {5, 19, 18, (uint8_t[7]){25, 33, 32, 26, 27, 14, 12}, 0xff};
#define PWMFREQ_LED 10000
#define PWMRES_LED 8
#define PWMCH_LED_ACTY 0
#define PWMCH_LED_ERR 1

/* Konfiguration siehe "Kconfig" im Library Ordner */

static const dmx_port_t DMXDRVR_PORT = DMX_NUM_2;
static const dmx_config_t DMXDRVR_CONFIG = DMX_CONFIG_DEFAULT;
#define DMX_PERS_COUNT 1
#define DMX_PERS_MAXCH 4
static dmx_personality_t DMX_PERSIES[DMX_PERS_COUNT] = {{4, "4xBtn 4ch"}};
static menuvar_t dmxdrvr_address = 1;
static menuvar_t dmxdrvr_pers_num = 1; /* Personality number */
static DmxBasicPers dmxdrvr_pers = DmxBasicPers(DMX_PERSIES, DMX_PERS_COUNT);
/* Tick for DMX heart beat */
#define TICK_DMX_HB 500

static HardwareSerial MidiSerial(1);     /* UART1 for MIDI */
static menuvar_t midi_channel = 16;      /* MIDI Channel */
static menuvar_t midi_firstNoteNo = 120; /* MIDI First Note Number */

#pragma endregion

#pragma region "GLOBAL VARIABLES"

/* Bitweiser Fehlercode */
static uint8_t errorCode = 0b0;
static menuvar_t errorCode_disp = 0;
/* Zeit seit Verbindungsabbruch (Sekunden) */
static menuvar_t sinceLostConnection = 0;
/* Nachrichten-Stempel Empfaenger. */
static stamp_ui32_t recv_stamp = 1;
/* Platzhalter Variable fuer Signalstaerke des Peers in Prozent. */
static menuvar_t rssiValPeer_perc;
/* Signalstaerke des Peers. */
static RssiValue<menuvar_t> rssiPeer = RssiValue<menuvar_t>(0, &rssiValPeer_perc, RSSI_NO_CONN, -50);
/* Speichert ob ein Taster betaetigt ist. */
// static uint8_t menubtnsPressed = 0b0000;
/* Menuetaster Aktion */
static MENUBTN_STATE menubtn_states[MENUBTN__COUNT] = {MENUBTN_STATE_NONE, MENUBTN_STATE_NONE, MENUBTN_STATE_NONE};
/* Speichert die Funktionskanaele. */
static uint8_t fun_channels[DMX_PERS_MAXCH] = {0};
/* Anzahl der aktuell verwendeteten Funktions-Taster (Modus). */
static const uint8_t FUNBTN_ANZ = 4;
/* Letzten Tasterdruck vom Sender zwischenspeichern fuer die Anzeige. */
static menuvar_t xmtr_lastFunBtn = DISPVAL_T_UNKNOWN;
/* Merker bei Blinken Displaymenue ob Prefix gerade gezeigt */
static bool disp_editArrow = false;
/* Verwaltung der Flash Einstellungen */
static Preferences preferences;
/* Merker um DMX Senden sofort auszuloesen */
static bool dmx_sendNow = false;
/* Currently show menu name in display. */
static bool showMenuName = false;

/* - TIME STAMPS - */

/* Timestamp last 4Hz tick */
unsigned long ts_tick4hz = 0;
/* Timestamp last display value update */
unsigned long ts_dispLastValUpdt = 0;
/* Timestamp tick for edit arrow blinking */
unsigned long ts_dispEditBlink = 0;
/* Timestamp start of showing menu name in display */
unsigned long ts_dispName = 0;
/* Timestamp last recognised button press/release */
unsigned long ts_lastBtnEvent = 0;
/* Timestamp since button is held down */
unsigned long ts_btnHolding = 0;
/* Timestamp last Alive message has been send */
unsigned long ts_aliveSend = 0;
/* Timestamp last message has been received */
unsigned long ts_activity = 0;
/* Timestamp since last connection loss (0: RESET STATE!) */
unsigned long ts_lostConnection = 0UL;
/* Timestamp last sending of DMX packet has been started */
unsigned long ts_dmxSending = 0;

/* DISPLAY VARIABLEN */

#define MENU_PFIX_ADRS 'A' /* Menu Key ADRS */
#define MENU_PFIX_MCHA 'C' /* Menu Key MCHA */
#define MENU_PFIX_NOTE 'N' /* Menu Key MCC# */
#define MENU_PFIX_VRBG 'V' /* Menu Key VRBG */
#define MENU_PFIX_TAST 'T' /* Menu Key TAST */
std::vector<rcvr_disp_menue_t> disp_menus = {
    {'S', (char *)"SIG%", false, &rssiValPeer_perc, 0, 100}, /* Signal strength */
    {'P', (char *)"PERS", true, &dmxdrvr_pers_num, 1, DMX_PERS_COUNT},
    /* Personality (Channel mode) */                                        // TODO
    {MENU_PFIX_ADRS, (char *)"ADRS", true, &dmxdrvr_address, 1, 512},       /* Address */
    {MENU_PFIX_MCHA, (char *)"MCHA", true, &midi_channel, 1, 16},           /* MIDI Channel */
    {MENU_PFIX_NOTE, (char *)"NOTE", true, &midi_firstNoteNo, 0, 127},      /* MIDI First Note */
    {MENU_PFIX_TAST, (char *)"TAST", false, &xmtr_lastFunBtn, -6, 6, 0b10}, /* Last function button */
    {MENU_PFIX_VRBG, (char *)"VRBG", false, &sinceLostConnection, 1, 900},  /* Connection (0: reset state) */
    {'F', (char *)"FEHL", false, &errorCode_disp, 0, 63}                    /* Errorcode */
};
static const uint8_t MENU_COUNT = disp_menus.size();
rcvr_disp_menue_t *const DISPMENU_ADRS = &disp_menus[2];
DispSlx2016_4d Disp1 = DispSlx2016_4d(&disp_menus, MENU_COUNT, &OUT_DISP);

#pragma endregion

#pragma region "Declarations"

void sendDmxData();
void setErrorcodeBit(SOFU_ERRORCODE_BIT fc_bit, bool set);
void resetDispTimestamps();
void MenuBtnPress(uint8_t menubtnsPressed, uint8_t ixBtn);
void MenuBtnHold(uint8_t menubtnsPressed, uint8_t ixBtn);
void DataRecv_Rcvr(const uint8_t *mac, const uint8_t *in_data, int data_len);
void PromiscuousRxEspnow_Rcvr(void *buf, wifi_promiscuous_pkt_type_t type);
bool SaveUserSetting_Disp(rcvr_disp_menue_t currMenu);

void ARDUINO_ISR_ATTR OnMenuBtnChange();

#pragma endregion

#pragma region "INITIALISATION FUNCTIONS"

void initEspNow_Rcvr() {
    Serial.print("[INFO] ESP-NOW init...");
    if ( esp_now_init() != ESP_OK ) {
        Serial.println("[ERROR] Could not initialise ESP-NOW.");
        setErrorcodeBit(ERRBIT_ENOW, true);
    } else {
        // Register peer
        esp_now_peer_info_t peerInfo = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
        memcpy(peerInfo.peer_addr, MAC_PEER, 6);
        // Add peer
        if ( esp_now_add_peer(&peerInfo) != ESP_OK ) {
            Serial.println("[ERROR] Cannot add peer connection.");
            setErrorcodeBit(ERRBIT_ENOW, true);
        }
        // Register callback function for ESP-NOW receiver
        esp_now_register_recv_cb(esp_now_recv_cb_t(DataRecv_Rcvr));
    }
    // Register callback function for signal strength reading
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(PromiscuousRxEspnow_Rcvr);
    Serial.println(" OK.");
}

void initDmxDriver_Rcvr() {
    Serial.print("[INFO] DMX driver init...");
    // install the DMX driver...
    if ( !dmx_driver_install(DMXDRVR_PORT, &DMXDRVR_CONFIG, DMX_PERSIES, DMX_PERS_COUNT) ) {
        Serial.println("[ERROR] DMX init: Error while installing DMX driver.");
        setErrorcodeBit(ERRBIT_DMX, true);
    } else {
        /* set custom communication pins: */
        dmx_set_pin(DMXDRVR_PORT, OUT_DMX_TX, OUT_DMX_RX, OUT_DMX_RTS);
        dmx_set_start_address(DMXDRVR_PORT, dmxdrvr_address);
    }
    Serial.println(" OK.");
}

void loadPrefs_Rcvr() {
    if ( preferences.begin(PREFS, false) ) {
        if ( digitalRead(IN_MENUBTNS[MENU_DN].pin) == HIGH && digitalRead(IN_MENUBTNS[MENU_UP].pin) == HIGH ) {
            /* Up+Down: Reset preferences */
            Disp1.writeChars("00 !");
            delay(1200);
            if ( preferences.clear() ) Disp1.writeChars(" OK ");
        }

        dmxdrvr_address = preferences.getShort(PREFS_DMX_ADRS, 401);
        dmxdrvr_pers.chooseByNum(preferences.getShort(PREFS_PERS_NUM, 1));
        dmxdrvr_pers_num = dmxdrvr_pers.num();
        DISPMENU_ADRS->varMax = (513 - dmxdrvr_pers.footpr());

        midi_channel = preferences.getShort(PREFS_MIDI_CHAN, 16);
        midi_firstNoteNo = preferences.getShort(PREFS_MIDI_NOTE, 120U);

        preferences.end();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nLAWA SONAR+FUNK R1 - RECEIVER starting");
    Serial.println("Firmware Ver " + String(FIRMWARE_VER) + "\n");

    /* PWM initialisieren */
    ledcSetup(PWMCH_LED_ACTY, PWMFREQ_LED, PWMRES_LED);
    ledcSetup(PWMCH_LED_ERR, PWMFREQ_LED, PWMRES_LED);
    /* Pins initialisieren: */
    ledcAttachPin(OUT_LED_ACTY, PWMCH_LED_ACTY);
    ledcAttachPin(OUT_LED_ERROR, PWMCH_LED_ERR);
    // gpio_config(IN_MENUBTNS_GPIO_CFG);
    gpio_config_t mneubtns_cfg = {
        .pin_bit_mask =
            ((1ULL << IN_MENUBTNS[MENU_DN].pin) | (1ULL << IN_MENUBTNS[MENU_UP].pin) |
             (1ULL << IN_MENUBTNS[MENU_OK].pin)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&mneubtns_cfg);

    /* LED Test Start: */
    ledcWrite(PWMCH_LED_ACTY, PWM_BRT);
    ledcWrite(PWMCH_LED_ERR, PWM_BRT);

    /* Display initialisation: */
    Disp1.init();
    Disp1.OnSaveUserSetting = &SaveUserSetting_Disp;

    /* ELoad preferences: */
    loadPrefs_Rcvr();

    /* WIFI initialisation: */
    Serial.println("[INFO] Initialising Wi-Fi... (Ignore error 0x300a WIFI_IF_STA not found)");
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    Serial.println(" OK.");
    /* Override MAC address: */
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, MAC_ME);
    if ( err == ESP_OK ) {
        static char macStr[MAC_STR_LEN];
        getMacAddrStr_esp(macStr);
        Serial.print("[ OK ] Changed MAC. MAC: " + String(macStr) + "\n");
    }

    /* ESP-NOW initialisation: */
    initEspNow_Rcvr();

    /* DMX driver initialisation: */
    initDmxDriver_Rcvr();

    /* MIDI Out initialisation: */
    Serial.print("[INFO] MIDI initialisation...");
    MidiSerial.begin(31250, SERIAL_8N1, OUT_MIDI_RX, OUT_MIDI_TX);
    if ( !MidiSerial ) {
        Serial.println("[ERROR] Could not open MIDI UART.");
        setErrorcodeBit(ERRBIT_MIDI, true);
    } else
        Serial.println(" OK.");

    delay(500);
    /* LED & Display Test: */
    char *welcome = SCROLLTEXT_MODELNAME;
    Disp1.setScrolltext(welcome, true);
    while ( Disp1.shiftTextByOne() )
        delay(120);
    Disp1.clear();

    Serial.println("[INFO] Initialisations finished.\n");
    delay(500);
    /* LED Test off: */
    ledcWrite(PWMCH_LED_ACTY, 0x0);
    ledcWrite(PWMCH_LED_ERR, 0x0);
    
    for ( menubtn_t btn : IN_MENUBTNS ) {
      attachInterrupt(btn.pin, OnMenuBtnChange, CHANGE);
    }

    /* Write initial DMX packet */
    for ( short b = 1; b <= 512; b++ )
        dmx_write_slot(DMXDRVR_PORT, b, 0x00);

    resetDispTimestamps();
    wait_after_setup += millis();
} /* setup */

#pragma endregion

#pragma region "MAIN FUNCTIONS"

void loop() {

    /* Taster abfragen: */
    bool bttg = false;
    for ( menubtn_t tstr : IN_MENUBTNS ) {
        switch ( menubtn_states[tstr.bit] ) {
        case MENUBTN_STATE_PRESSED:
            MenuBtnPress(menubtnsPressed, tstr.bit) menubtn_states[tstr.bit] = MENUBTN_STATE_NONE;
            break;
        case MENUBTN_STATE_HOLD:
            if ( millis() - ts_btnHolding >= TICK_BTN_HOLD ) MenuBtnHold(menubtnsPressed, tstr.bit);
            break;
        default:
            break;
        }
    }

    /* Sending DMX respecting packet duration: */
    if ( millis() - ts_dmxSending >= DMX_TICK_MIN ) {
        if ( dmx_sendNow || (millis() - ts_dmxSending >= TICK_DMX_HB) ) {
            sendDmxData();
        }
    }

    /* Activity Status LED: */
    if ( millis() - ts_activity >= TICK_LED_FLASH ) {
        ledcWrite(PWMCH_LED_ACTY, PWM_DIM);
    }

    /* Show name for determined time or value instead: */
    if ( millis() - ts_dispName < DUR_DISP_MNAME ) {
        if ( showMenuName ) {
            Disp1.showMenuName();
            showMenuName = false;
        }
    } else if ( Disp1.isEdit() && (millis() - ts_dispEditBlink >= DUR_DISP_EDIT[disp_editArrow ? 1 : 0]) ) {
        /* Show blinking arrow in display edit mode. */
        disp_editArrow = !disp_editArrow;
        Disp1.showValue(disp_editArrow);
        ts_dispEditBlink = millis();
    } else if ( millis() - ts_dispLastValUpdt >= DUR_DISP_UPDT ) {
        /* Update display. Set sinceLostConnection, where 0 is the reset state. */
        if ( Disp1.getCurrPrefix() == MENU_PFIX_VRBG )
            sinceLostConnection = (ts_lostConnection > 0 ? ((millis() - ts_lostConnection) / 1000) : 0);
        Disp1.showValue(disp_editArrow);
        xmtr_lastFunBtn = (errorCode == 0 ? DISPVAL_T_NONE : DISPVAL_T_UNKNOWN); /* Clear LastFunBtn */
        ts_dispLastValUpdt = millis();
    }

    /* Sending ALIVE heart beat: */
    if ( millis() - ts_aliveSend >= TICK_ALIVE ) {
        sofu_pkt_alive_t pkt = {recv_stamp++, false};
        EspNowSendMsg_SoFu(MAC_PEER, asbytesptr(&pkt), pkt.len);
        ts_aliveSend = millis();
    }

    /* Connection lost (only triggered once on lost connection): */
    if ( (millis() - ts_activity >= 2 * TICK_ALIVE) && !bitRead(errorCode, ERRBIT_NO_CONN) &&
         (millis() > wait_after_setup) ) {
        setErrorcodeBit(ERRBIT_NO_CONN, true);
        rssiPeer = RSSI_NO_CONN;
        xmtr_lastFunBtn = DISPVAL_T_UNKNOWN;
        ts_lostConnection = ts_dispLastValUpdt = millis();
        if ( millis() > DUR_ERR_LED_CONN ) Disp1.jumpToMenu(MENU_PFIX_VRBG);
        Serial.println("[WARN] Lost connection to transmitter!");
    }

    /* Do something with every 4Hz tick: */
    if ( millis() - ts_tick4hz >= TICK_4HZ ) {
        ts_tick4hz = millis();
        /* Error LED: */
        if ( bitRead(errorCode, ERRBIT_NO_CONN) )
            ledcWrite(PWMCH_LED_ERR, ledcRead(PWMCH_LED_ERR) == 0 ? PWM_FULL : 0x0); /* Connection now lost */
        else if (
            (millis() - ts_lostConnection < DUR_ERR_LED_CONN) && (millis() > wait_after_setup + DUR_ERR_LED_CONN) )
            ledcWrite(PWMCH_LED_ERR, PWM_DIM); /* Connection was lost not too long ago */
        else if ( errorCode > 0 )
            ledcWrite(PWMCH_LED_ERR, PWM_BRT); /* Other error */
        else
            ledcWrite(PWMCH_LED_ERR, 0x0); /* Clear error LED */
    }

#ifdef DEBUG
    Serial.printf("![DEBUG] Errorcode: %x \n", errorCode);
#endif
} /* loop */

/* DMX-Paket senden. */
void sendDmxData() {
    dmx_write_offset(DMXDRVR_PORT, dmxdrvr_address, fun_channels, dmxdrvr_pers.footpr());
    dmx_send(DMXDRVR_PORT);
    dmx_sendNow = false;
    ts_dmxSending = millis();
}

/*  */
void sendMidiData(sofu_pkt_funbtn_t pkt) {
    uint8_t midi_msg[3];
    midi_msg[0] = 0x90 | ((midi_channel - 1) & 0x0f);           /* Note On, Channel */
    midi_msg[1] = (midi_firstNoteNo + pkt.lastFunbtnId) & 0x7f; /* Note Number */
    if ( pkt.lastActn == FUNBTN_ACTION::FUNBTN_BTNDOWN ) {
        midi_msg[2] = 0x7f; /* Velocity 100% */
    } else if ( pkt.lastActn == FUNBTN_ACTION::FUNBTN_BTNUP ) {
        midi_msg[2] = 0x00; /* Velocity 0% */
    } else
        return;
    MidiSerial.write(midi_msg, 3);
}

/* Setzt den Fehlercode des Empfaengers. */
void setErrorcodeBit(SOFU_ERRORCODE_BIT fc_bit, bool set) {
    bitWrite(errorCode, fc_bit, set);
    bitWrite(errorCode_disp, fc_bit, set);
}

void resetDispTimestamps() {
    ts_dispName = millis();
    showMenuName = true;
}

#pragma endregion

#pragma region "INTERRUPTS"

void ARDUINO_ISR_ATTR OnMenuBtnChange() {
    for ( menubtn_t tstr : IN_MENUBTNS ) {
        if ( digitalRead(tstr.pin) == HIGH ) {
            menubtn_states[tstr.bit] = MENUBTN_STATE_DOWN;

        } else if ( menubtn_states[tstr.bit] == MENUBTN_STATE_DOWN ) {
            if ( millis() - ts_lastBtnEvent >= DUR_BTN_HYST ) {
                /* Pos Flanke = Taster Betaetigung, entprellt um DUR_BTN_HYST */
                if ( millis() - ts_lastBtnEvent >= DUR_BTN_HOLD )
                    menubtn_states[tstr.bit] = MENUBTN_STATE_HOLD;
                else
                    menubtn_states[tstr.bit] = MENUBTN_STATE_PRESSED;
            }
        } else {
            menubtn_states[tstr.bit] = MENUBTN_STATE_NONE;
        }
    }
    ts_lastBtnEvent = millis();
}

#pragma endregion

#pragma region "CALLBACKS"

/* Callback Taster Druecken. Wird nur EINMALIG aufgerufen wenn Taster KURZ BETAETIGT. */
void MenuBtnPress(uint8_t menubtnsPressed, uint8_t ixBtn) {
    if ( Disp1.isEdit() ) {
        if ( (ixBtn == IN_MENUBTNS[MENU_DN].bit) && !bitRead(menubtnsPressed, IN_MENUBTNS[MENU_UP].bit) ) {
            /* Taster Runter betaetigt */
            Disp1.valueDn();
        } else if ( (ixBtn == IN_MENUBTNS[MENU_UP].bit) && !bitRead(menubtnsPressed, IN_MENUBTNS[MENU_DN].bit) ) {
            /* Taster Hoch betaetigt */
            Disp1.valueUp();
        } else if ( ixBtn == IN_MENUBTNS[MENU_OK].bit ) {
            /* Taste OK: Speichern */
            /* Anzeige des Text "OK" fuer Anzeigedauer: */
            Disp1.saveSetting();
            disp_editArrow = false;
            ts_dispLastValUpdt = millis();
        }
    } else {
        /* Anzeigemodus: */
        if ( ixBtn == IN_MENUBTNS[MENU_DN].bit ) {
            /* Taster DN kurz betaetigt: Vorheriges Menue */
            Disp1.prevMenu();
            resetDispTimestamps();
        } else if ( ixBtn == IN_MENUBTNS[MENU_UP].bit ) {
            /* Taster UP kurz betaetigt: Naechstes Menue */
            Disp1.nextMenu();
            resetDispTimestamps();
        } else if ( ixBtn == IN_MENUBTNS[MENU_OK].bit ) {
            /* Taster OK kurz betaetigt */
            if ( Disp1.getCurrPrefix() == MENU_PFIX_VRBG ) {
                if ( !bitRead(errorCode, ERRBIT_NO_CONN) ) {
                    ts_lostConnection = 0UL; /* Reset lost connection info status */
                    Disp1.writeChars(" 00 ");
                } else
                    Disp1.writeChars("  X ");
                ts_dispLastValUpdt = millis();
            } else {
                Disp1.showMenuName();
                resetDispTimestamps();
            }
        }
    }
}

/* Callback Taster Halten. */
void MenuBtnHold(uint8_t menubtnsPressed, uint8_t ixBtn) {
    if ( menubtn_states[MENU_OK] ) {
        /* OK gehalten, Einstellmodus wechseln */
        Disp1.toogleEdit();
        menubtn_states[MENU_OK] = MENUBTN_STATE_NONE; /* Reset state fuer OK. */
    } else if ( Disp1.isEdit() ) {
        /* Wenn Display im Einstellmodus */
        if ( (menubtn_states[MENU_DN]) && (menubtn_states[MENU_UP] == 0) ) {
            /* Taster Runter exklusiv gedrueckt */
            Disp1.valueDn(true);
        } else if ( (menubtn_states[MENU_UP]) && (menubtn_states[MENU_DN] == 0) ) {
            /* Taster Hoch exklusiv gedrueckt */
            Disp1.valueUp(true);
        }
    }
    ts_btnHolding = millis();
}

/* Callback Empfangenes Paket verarbeiten. */
void DataRecv_Rcvr(const uint8_t *mac, const uint8_t *in_data, int data_len) {
    if ( !sofu_pkt_primiv_t::valides_pkt(in_data, data_len) ) {
        Serial.println("[INFO] ESP-NOW: Paket mit unbekanntem Format empfangen.");
        return;
    }
    ts_activity = millis();
    ledcWrite(PWMCH_LED_ACTY, PWM_BRT);

#if defined DEBUG || defined DEBUG_WIREL
    for ( short i = 0; i < data_len; i++ )
        Serial.printf("%hhx\n", in_data[i]);
#endif
    if ( in_data[SOFU_PKT_KEY_END] == PKT_TYPE_ALIVE ) {
        /* Alive Paket, Verbindung wiederhergestellt: */
        if ( bitRead(errorCode, ERRBIT_NO_CONN) ) Serial.println("[INFO] Connection reestablished.");
        setErrorcodeBit(ERRBIT_NO_CONN, false);
        ts_activity = millis();
    } else if ( in_data[SOFU_PKT_KEY_END] == PKT_TYPE_FUNBTN ) {
        /* F-Taster Paket */
        sofu_pkt_funbtn_t pktbtn = {};
        memcpy(&pktbtn, in_data, sizeof(sofu_pkt_funbtn_t));
        /* Betaetigung uint8_t F-Taster auslesen */
        for ( uint8_t i = 0; i < FUNBTN_ANZ; i++ ) {
            fun_channels[i] = bitRead(pktbtn.btn_byte, i) ? 0xff : 0x00;
        }
        /* Letzten F-Taster auslesen, Index und Betaetigung: */
        switch ( pktbtn.lastActn ) {
        case FUNBTN_BTNDOWN:
            xmtr_lastFunBtn = -(pktbtn.lastFunbtnId + 1);
            break;
        case FUNBTN_BTNUP:
            xmtr_lastFunBtn = (pktbtn.lastFunbtnId + 1);
            break;
        case FUNBTN_ACTN_REPEAT:
            xmtr_lastFunBtn = DISPVAL_T_INFO;
            break;
        default:
            xmtr_lastFunBtn = DISPVAL_T_UNKNOWN;
        }
        ts_dispLastValUpdt = millis() - DUR_DISP_UPDT; /* Update now! */
        dmx_sendNow = true;
        sendMidiData(pktbtn);
#ifdef DEBUG_WIREL
        /* Monitor empfangene Taste: */
        Serial.println(pktbtn.toString());
#endif
    }
}

/* Callback WiFi Rx Pakete (Signalqualitaet) verarbeiten. */
void PromiscuousRxEspnow_Rcvr(void *buf, wifi_promiscuous_pkt_type_t type) {
    /* All espnow traffic uses action frames which are a subtype of the mgmnt
     * frames so filter out everything else. */
    if ( type != WIFI_PKT_MGMT ) return;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    /* Nur weiter verarbeiten wenn Action Frame und passende OUI */
    if ( (hdr->frame_ctrl & 0xFF) == ACTION_SUBTYPE && memcmp(hdr->addr1, MACOUI_SF, sizeof(MACOUI_SF)) == 0 ) {
        rssiPeer = ppkt->rx_ctrl.rssi;
#if defined DEBUG || defined DEBUG_WIREL
        Serial.println("![DEBUG] Signal: " + (String)rssiPeer + " " + rssiPeer.prozStr());
#endif
    }
}

/* Callback Display Einstellung bestaetigt */
bool SaveUserSetting_Disp(rcvr_disp_menue_t currMenu) {
    if ( preferences.begin(PREFS, false) ) {
        /* Save settings to flash: */
        preferences.putShort(PREFS_DMX_ADRS, dmxdrvr_address);
        preferences.putShort(PREFS_PERS_NUM, dmxdrvr_pers_num);
        preferences.putShort(PREFS_MIDI_CHAN, midi_channel);
        preferences.putShort(PREFS_MIDI_NOTE, midi_firstNoteNo);
        preferences.end();
        /* Choose new personality and set address: */
        if ( !dmxdrvr_pers.chooseByNum(dmxdrvr_pers_num) ) return false;
        if ( !dmx_set_current_personality(DMXDRVR_PORT, dmxdrvr_pers.num()) ) return false;
        if ( !dmx_set_start_address(DMXDRVR_PORT, dmxdrvr_address) ) return false;
        /* Set new max for address menu: */
        DISPMENU_ADRS->varMax = (513 - dmxdrvr_pers.footpr());
        return true;
    } else
        return false;
}

#pragma endregion