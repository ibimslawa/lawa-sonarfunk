
#include "esp_dmx.h"
#include "lawa_sonarfunk_lib.h"

/* Typklasse fuer die Verwendung einer dmx_personality_t anhand eines Int.
 * Verwendet Wert in dBm, wenn verwendet wie eine Variable. Member perc gibt Wert in Prozent an. */
class DmxBasicPers {
private:
  unsigned int _num = 1;
  unsigned int pers_count = 1;
  const dmx_personality_t *pers_arr;

public:
  /* Definiert ein DmxBasicPers Objekt mit einem Personality Array und Pers Anzahl. */
  DmxBasicPers(const dmx_personality_t *pers_arr, const int pers_count) {
    this->pers_arr = pers_arr;
    this->pers_count = pers_count;
  }

  dmx_personality_t pers() {
    return pers_arr[_num - 1];
  }
  virtual operator dmx_personality_t() const {
    return pers_arr[_num - 1];
  }
  int footpr() {
    return pers_arr[_num - 1].footprint;
  }
  int num() {
    return _num;
  }
  bool chooseByNum(unsigned short num_new) {
    if ( num_new >= 1 && num_new <= pers_count )
      _num = num_new;
    else
      return false;
    return true;
  }
  bool chooseByFootp(unsigned short fp_new) {
    for ( int i = 1; i <= pers_count; i++ )
      if ( pers_arr[i - 1].footprint == fp_new ) {
        _num = i;
        return true;
      }
    return false;
  }
};
