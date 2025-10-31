/* Lars Warich Sonar+Funk R1 (Revision 1)
 * DMX Taster-Fernbedienung.
 * DISPLAY BIBLIOTHEK (EMPFAENGER-MODUL)
 * Ver 25-03-14 */

#pragma once

#include "lawa_sonarfunk_lib.h"
#include <vector>

using namespace std;

/* Ziffern des Displays. */
#define DISP_ZIFF 4

typedef int16_t menuvar_t;
enum dispval_t_specials : int16_t {
  DISPVAL_T_NONE = (INT16_MIN),
  DISPVAL_T_UNKNOWN = (INT16_MAX),
  DISPVAL_T_INFO = (INT16_MIN + 1)
};

struct rcvr_disp_menue_t {
  char *name;
  char prefix = '#';
  /* Steuert ob die Menuevariable editierbar ist */
  bool editable = false;
  /* Variable der Einstellung des Menues (Pointer) */
  menuvar_t *menuVar_p = nullptr;
  /* Minimum der Variable der Einstellung des Menues */
  menuvar_t varMin = 0;
  /* Maxmium der Variable der Einstellung des Menues */
  menuvar_t varMax = 100;
  /* Controls the display of the sign. 0: minus, 1: plusminus, 2+: arrow at digit */
  uint8_t plusmin = false;

  rcvr_disp_menue_t(char prefix, char *name, bool editable, menuvar_t *var_p, menuvar_t varMin, menuvar_t varMax) {
    this->name = (char *)malloc(strlen(name) + 1);
    this->prefix = prefix, strcpy(this->name, name), this->editable = editable, this->menuVar_p = var_p,
    this->varMin = varMin, this->varMax = varMax;
  }
  rcvr_disp_menue_t(
      char prefix, char *name, bool editable, menuvar_t *var_p, menuvar_t varMin, menuvar_t varMax, uint8_t plusmin)
      : rcvr_disp_menue_t(prefix, name, editable, var_p, varMin, varMax) {
    this->plusmin = plusmin;
  }
};

struct slx2016_d4_pins_t {
  // Write Pin (invertiert)
  uint8_t wr_ = 0;
  // Digit Select Pin A0
  uint8_t a0 = 0;
  // Digit Select Pin A1
  uint8_t a1 = 0;
  // Digit Data Pins 7-bit
  uint8_t d[7] = {0};
  // Blank Display Pin (invertiert)
  uint8_t bl_ = -1;
  // Pointer array auf alle Pins
  uint8_t *pins[11] = {&wr_, &a0, &a1, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &bl_};

  slx2016_d4_pins_t(uint8_t wr_, uint8_t a0, uint8_t a1, uint8_t _d[7], uint8_t bl_)
      : wr_(wr_), a0(a0), a1(a1), bl_(bl_) {
    memcpy(d, _d, sizeof(d));
  }
};

#define SKIP_DIGIT (char)0xff
enum slx2016_special_char : char {
  SLX_ARROWUP = 0x01,
  SLX_ARROWRI = 0x02,
  SLX_ARROWDN = 0x03,
  SLX_ARROWLE = 0x04,
  SLX_CELSIUS = 0x1b,
  SLX_FAHRENH = 0x1c,
  SLX_BLOB = 0x7f
};

class DispSlx2016_4d {

public:
  vector<rcvr_disp_menue_t> *menus = nullptr;
  menuvar_t tempVar = 0;
  const slx2016_d4_pins_t *pin_def;
  /* Callback fuer das Speichern der Einstellung. */
  bool (*OnSaveUserSetting)(rcvr_disp_menue_t currMenu);
  static const int16_t SLX_4D_MIN = -99, SLX_4D_MAX = 999;
  static const uint8_t DIGITS = 4;

private:
  uint8_t menuId = 0;
  uint8_t MENU_COUNT = 0;
  /* Aktuelles Menue */
  rcvr_disp_menue_t* currMenu;
  /* Einstellmodus aktiv */
  bool editMode = false;
  char *scrolltext;
  size_t scrolltext_maxix = 0;
  size_t scrolltext_ix = 0;

public:
  DispSlx2016_4d(vector<rcvr_disp_menue_t> *const menus, unsigned short MENU_COUNT, const slx2016_d4_pins_t *pin_def) {
    this->menus = menus;
    this->MENU_COUNT = MENU_COUNT;
    this->pin_def = pin_def;
    currMenu = &menus->front();
    tempVar = *(currMenu->menuVar_p);
  };

  /* Initialisiert das Display. Setzt die Pin Modes  */
  void init() {
    for ( uint8_t *pin : pin_def->pins )
      if ( *pin > 0 && *pin <= ESP_DEV4_GPIO_MAX ) pinMode(*pin, OUTPUT);
    uint8_t blobs[DIGITS] = {SLX_BLOB, SLX_BLOB, SLX_BLOB, SLX_BLOB};
    writeChars(blobs);
  }
  /* Display clear. */
  void clear() {
    writeChars("    ");
    // digitalWrite(pin_def->bl_, LOW); delay(1); digitalWrite(pin_def->bl_, HIGH);
  }
  /* Waehlt das naechste Menue aus. Einstellen wird abgebrochen. Zeigt Menuenamen an. */
  void nextMenu() {
    if ( menuId >= MENU_COUNT - 1 )
      goToMenu(0);
    else
      goToMenu(++menuId);
  }
  /* Waehlt das vorherige Menue aus. Einstellen wird abgebrochen. Zeigt Menuenamen an. */
  void prevMenu() {
    if ( menuId <= 0 )
      goToMenu(MENU_COUNT - 1);
    else
      goToMenu(--menuId);
  }
  /* Returns a menu index for a specified prefix char. */
  rcvr_disp_menue_t *const getMenu(char prefix) {
    for ( uint8_t ix = 0; ix < MENU_COUNT; ix++ )
      if ( menus->at(ix).prefix == prefix ) return &(menus->at(ix));
  }
  /* */
  uint8_t getCurrId() {
    return menuId;
  }
  /* */
  char getCurrPrefix() {
    return currMenu->prefix;
  }
  /* */
  bool isEdit() {
    return editMode;
  }
  bool jumpToMenu(char prefix) {
    if ( currMenu->prefix == prefix ) return true;
    for ( uint8_t ix = 0; ix < MENU_COUNT; ix++ )
      if ( menus->at(ix).prefix == prefix ) {
        goToMenu(ix);
        return true;
      }
    return false;
  }
  /* Steuert ob das Menue sich im Einstellmodus befindet. Bei Verlassen wird der Wert wiederhergestellt. */
  void setEdit(bool _edit) {
    if ( editMode && !_edit ) tempVar = *(currMenu->menuVar_p);
    if ( currMenu->editable ) editMode = _edit;
  }
  /*  */
  void toogleEdit() {
    setEdit(!editMode);
  }
  /* Speichert die Einstellung und verlaesst den Modus.
   * Callback OnSaveUserSetting wird aufgerufen - zeigt "OK" oder "!!" anhand bool return an. */
  void saveSetting() {
    *(currMenu->menuVar_p) = tempVar;
    editMode = false;
    if ( OnSaveUserSetting(*currMenu) )
      writeChars(" OK ");
    else
      writeChars(" !! ");
  }
  /* Wert der Einstellung verringern */
  void valueDn(bool halten = false) {
    if ( editMode ) {
      if ( tempVar > currMenu->varMin ) {
        tempVar--;
      } else if ( !halten )
        tempVar = currMenu->varMax;
    }
    showValue(SKIP_DIGIT);
  }
  /* Wert der Einstellung erhoehen */
  void valueUp(bool halten = false) {
    if ( editMode ) {
      if ( tempVar < currMenu->varMax ) {
        tempVar++;
      } else if ( !halten )
        tempVar = currMenu->varMin;
    }
    showValue(SKIP_DIGIT);
  }
  /* Zeigt den Namen des aktuellen Menues an. */
  void showMenuName() {
    writeChars(currMenu->name);
  }
  /* Zeigt das Menue mit Prefix und Wert an. */
  void showValue(bool editArrow = false) {
    showValue(editArrow ? '>' : currMenu->prefix);
  }
  /* Shows the digits on display. Set digit to SKIP_DIGIT (0xff) to skip this digit. */
  void writeChars(uint8_t digits[DIGITS]) {
    writeChars((const char *)digits);
  }
  /* Shows the text or substring on display. Set digit to SKIP_DIGIT (0xff) to skip this digit. */
  void writeChars(const char *str, size_t offset = 0U) {
    int a = DISP_ZIFF - 1;
    for ( size_t dig = offset; dig < (DISP_ZIFF + offset); dig++ ) {
      if ( str[dig] != SKIP_DIGIT ) {
        digitalWrite(pin_def->a0, bitRead(a, 0) ? HIGH : LOW);
        digitalWrite(pin_def->a1, bitRead(a, 1) ? HIGH : LOW);
        digitalWrite(pin_def->wr_, LOW);
        for ( uint8_t pD = 0; pD < 7; pD++ ) {
          digitalWrite(pin_def->d[pD], bitRead(str[dig], pD) ? HIGH : LOW);
        }
        digitalWrite(pin_def->wr_, HIGH);
      }
      a--;
    }
  }
  /**/
  void setScrolltext(const char *text, bool whitespace = false) {
    if ( this->scrolltext == nullptr ) free(this->scrolltext);
    size_t len;
    if ( whitespace ) {
      len = strlen(text) + 2 * DIGITS;
      scrolltext = (char *)malloc(len + 1);
      strcpy(&scrolltext[DIGITS], text);
      for ( size_t i = 0; i < DIGITS; i++ ) {
        scrolltext[i] = ' ';
        scrolltext[len - i - 1] = ' ';
      }
    } else {
      len = strlen(text);
      scrolltext = (char *)malloc(len + 1);
      strcpy(scrolltext, text);
    }
    scrolltext_maxix = len - DIGITS;
  }
  /* Scrolls the set scrolltext by one character (returns true).
   * Returns false if no characters are left to scroll. */
  bool shiftTextByOne() {
    if ( scrolltext_ix < scrolltext_maxix ) {
      writeChars(scrolltext, scrolltext_ix);
      scrolltext_ix++;
      return true;
    } else
      return false;
  }

private:
  /* Springt zu einem bestimmten Menue an einem Index. */
  void goToMenu(uint8_t ix) {
    editMode = false;
    menuId = ix;
    currMenu = &(menus->at(menuId));
    tempVar = *(currMenu->menuVar_p);
    showMenuName();
  }
  void showValue(char first) {
    if ( !editMode ) tempVar = *(currMenu->menuVar_p);
    char str[DIGITS + 1];
    switch ( tempVar ) {
      break;
    case DISPVAL_T_NONE:
      strcpy(str, "$...");
      break;
    case DISPVAL_T_INFO:
      strcpy(str, "$ **");
      break;
    case DISPVAL_T_UNKNOWN:
      strcpy(str, "$ ??");
      break;
    default:
      if ( tempVar < currMenu->varMin || tempVar > currMenu->varMax )
        strcpy(str, "$...");
      else if ( tempVar < SLX_4D_MIN )
        strcpy(str, "$ ++");
      else if ( tempVar < SLX_4D_MIN )
        strcpy(str, "$ --");
      else if ( currMenu->plusmin == 0b0 )
        sprintf(str, "$%3d", tempVar);
      else if ( currMenu->plusmin == 0b1 )
        sprintf(str, "$%+3d", tempVar);
      else if ( currMenu->plusmin == 0b10 ) {
        if ( tempVar > 0 )
          sprintf(str, "$ %c%1d", SLX_ARROWUP, tempVar);
        else if ( tempVar < 0 )
          sprintf(str, "$ %c%1d", SLX_ARROWDN, -tempVar);
        else
          sprintf(str, "$ %c0", SLX_ARROWRI);
      }
      else {
        if ( tempVar > 0 )
          sprintf(str, "$%c%2d", SLX_ARROWUP, tempVar);
        else if ( tempVar < 0 )
          sprintf(str, "$%c%2d", SLX_ARROWDN, -tempVar);
        else
          sprintf(str, "$%c 0", SLX_ARROWRI);
      }
    }
    str[0] = first;
    writeChars(str);
  }
};
