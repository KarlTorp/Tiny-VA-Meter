#include "button.hpp"

BUTTON::BUTTON(uint8_t pin, bool pull_up, uint8_t pressed_state, uint16_t long_press_time, uint16_t debounce_time)
: m_pin(pin),
  m_long_press_time(long_press_time),
  m_debounce_time(debounce_time),
  m_pressed_state(pressed_state),
  m_button_state_set_time(0),
  m_button_state(STATE_NONE)
{
  if(pull_up) {
    pinMode(pin, INPUT_PULLUP); 
  } else {
    pinMode(pin, INPUT); 
  }
}

BUTTON::t_button_state BUTTON::updateState()
{
  const uint8_t buttonState = digitalRead(m_pin);
  const unsigned long currentMillis = millis();
  switch(m_button_state) {
    case STATE_NONE:
      if(buttonState == m_pressed_state && currentMillis > m_button_state_set_time + (unsigned long)m_debounce_time) {
        m_button_state = STATE_PRESSED;
        m_button_state_set_time = currentMillis;
      }
      break;
    case STATE_PRESSED:
      if(buttonState == m_pressed_state) {
        if(m_button_state_set_time + (unsigned long)m_long_press_time < currentMillis) {
          m_button_state = STATE_PRESSED_LP;
        }
      } else {
        m_button_state = STATE_RELEASED_SP;
        m_button_state_set_time = currentMillis;
      }
      break;
    case STATE_PRESSED_LP:
      if(buttonState != m_pressed_state) {
        m_button_state = STATE_RELEASED_LP;
        m_button_state_set_time = currentMillis;
      }
      break;
      case STATE_RELEASED_SP:
      case STATE_RELEASED_LP:
      default:
      m_button_state = STATE_NONE;
      break;
  }
  return m_button_state; 
}
  

