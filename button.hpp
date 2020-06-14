#include "Arduino.h"

class BUTTON
{
  public:
  enum t_button_state{
      STATE_NONE = 0,
      STATE_PRESSED,
      STATE_PRESSED_LP,
      STATE_RELEASED_SP,
      STATE_RELEASED_LP,
    };

  BUTTON(uint8_t pin, bool pull_up, uint8_t pressed_state, uint16_t long_press_time, uint16_t debounce_time);
  t_button_state updateState();

  private:
  uint8_t m_pin;
  uint16_t m_long_press_time;
  uint16_t m_debounce_time;
  uint8_t m_pressed_state;
  unsigned long m_button_state_set_time;
  t_button_state m_button_state;
};

