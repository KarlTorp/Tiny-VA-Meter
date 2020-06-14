#pragma once
#include <avr/pgmspace.h>

#define power_jack_icon_width 61
#define power_jack_icon_height 13
static const unsigned char PROGMEM power_jack_icon_bits[] = {
   0xf0, 0x01, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0x01, 0x1c, 0x07, 0x00, 0x38,
   0x06, 0x00, 0x1c, 0x07, 0x06, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x06, 0x0c,
   0x02, 0x08, 0x00, 0x04, 0x00, 0x00, 0x02, 0x08, 0x03, 0x18, 0x00, 0x86,
   0x00, 0x00, 0x43, 0x18, 0x01, 0x10, 0x00, 0xc2, 0x01, 0x00, 0x41, 0x10,
   0xf1, 0xf1, 0xff, 0xe3, 0xff, 0xff, 0xf1, 0x11, 0x01, 0x10, 0x00, 0xc2,
   0x01, 0x00, 0x41, 0x18, 0x03, 0x18, 0x00, 0x86, 0x00, 0x00, 0x41, 0x18,
   0x02, 0x08, 0x00, 0x04, 0x00, 0x00, 0x03, 0x08, 0x06, 0x0c, 0x00, 0x0c,
   0x00, 0x00, 0x06, 0x0c, 0x1c, 0x07, 0x00, 0x38, 0x06, 0x00, 0x1c, 0x07,
   0xf0, 0x01, 0x00, 0xe0, 0x03, 0x00, 0xf0, 0x01 
};

#define USB_icon_width 31
#define USB_icon_height 14
static const unsigned char PROGMEM USB_icon_bits[] = {
   0x00, 0x00, 0x0c, 0x00, 0x00, 0xe0, 0x1f, 0x00, 0x00, 0x30, 0x0c, 0x00,
   0x00, 0x18, 0x00, 0x00, 0x0e, 0x0c, 0x00, 0x00, 0x1f, 0x06, 0x00, 0x10,
   0xff, 0xff, 0xff, 0x7f, 0x1f, 0x60, 0x00, 0x10, 0x0e, 0xc0, 0x00, 0x00,
   0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xc6, 0x01,
   0x00, 0x00, 0xfc, 0x01, 0x00, 0x00, 0xc0, 0x01 
};

#ifdef ENABLE_TERMINAL
const char terminal_help_head[] PROGMEM =     {"Commands:"};
const char terminal_help_reset[] PROGMEM =    {"- reset (Reset mAh)"};
const char terminal_help_read[] PROGMEM =     {"- read (Reply with latest results)"};
const char terminal_help_log[] PROGMEM =      {"- log x (Auto tx of sampels - x can be on or off)"};
const char terminal_help_sleep[] PROGMEM =    {"- sleep x (INA219 sleep between samples - x can be on or off)"};
const char terminal_help_refresh[] PROGMEM =  {"- refresh x (Set screen & serial refresh rate. x can be 100, 200, 500 or 1000)"};
const char terminal_help_range_v[] PROGMEM =  {"- volt x (Set INA219 voltage range. x can be 16 or 32)"};
const char terminal_help_range_i[] PROGMEM =  {"- gain x (Set INA219 V&A adc 1/gain. x can be 1, 2, 4 or 8)"};
const char terminal_help_averaging[] PROGMEM =  {"- avg x (Set INA219 V&A averaging. x can be 1, 2, 4, 8, 16, 32, 64 or 128)"};
#endif
const char input_range_text[] PROGMEM =       {"Input range:"};
const char range_for_jack_text[] PROGMEM =    {"0-26V 3.2A"};
const char range_for_usb_text[] PROGMEM =     {"4-15V 3.2A"};
const char settings_text[] PROGMEM =          {"Settings"};
const char sensor_sleep_text[] PROGMEM =      {"Sensor sleep"};

const char* const string_table[] PROGMEM = {
#ifdef ENABLE_TERMINAL
  terminal_help_head,
  terminal_help_reset,
  terminal_help_read,
  terminal_help_log,
  terminal_help_sleep,
  terminal_help_refresh,
  terminal_help_range_v,
  terminal_help_range_i,
  terminal_help_averaging,
#endif
  input_range_text, 
  range_for_jack_text,
  range_for_usb_text,
  settings_text,
  sensor_sleep_text
};

enum FlashStrings {
#ifdef ENABLE_TERMINAL
  TXT_TERMINAL_HELP_HEAD,
  TXT_TERMINAL_HELP_RESET,
  TXT_TERMINAL_HELP_READ,
  TXT_TERMINAL_HELP_LOG,
  TXT_TERMINAL_HELP_SLEEP,
  TXT_TERMINAL_HELP_REFRESH,
  TXT_TERMINAL_HELP_RANGE_V,
  TXT_TERMINAL_HELP_RANGE_I,
  TXT_TERMINAL_HELP_AVG,
#endif
  TXT_INPUT_RANGE,
  TXT_JACK_RANGE,
  TXT_USB_RANGE,
  TXT_SETTINGS,
  TEXT_SENSOR_SLEEP
};

char buffer[80]; // Must be able to hold longest flash text string.

char* getFlashString(int i){
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[i])));
}
