#pragma once
#include <avr/pgmspace.h>

const char terminal_help_head[] PROGMEM =     {"Commands:"};
const char terminal_help_reset[] PROGMEM =    {"- reset (Reset mAh)"};
const char terminal_help_read[] PROGMEM =     {"- read (Reply with latest results)"};
const char terminal_help_log[] PROGMEM =      {"- log x (Auto tx of sampels - x can be on or off)"};
const char terminal_help_sleep[] PROGMEM =    {"- sleep x (INA219 sleep between samples - x can be on or off)"};
const char terminal_help_refresh[] PROGMEM =  {"- refresh x (Set screen & serial refresh rate. x can be 100, 200, 500 or 1000)"};
const char terminal_help_range[] PROGMEM =    {"- range x (Set INA219 range. x can be 0 for 3.2A, 1 for 1A or 2 for 0.4A)"};

const char* const string_table[] PROGMEM = {
  terminal_help_head,
  terminal_help_reset,
  terminal_help_read,
  terminal_help_log,
  terminal_help_sleep,
  terminal_help_refresh,
  terminal_help_range
};

enum FlashStrings {
  TXT_TERMINAL_HELP_HEAD = 0,
  TXT_TERMINAL_HELP_RESET,
  TXT_TERMINAL_HELP_READ,
  TXT_TERMINAL_HELP_LOG,
  TXT_TERMINAL_HELP_SLEEP,
  TXT_TERMINAL_HELP_REFRESH,
  TXT_TERMINAL_HELP_RANGE
};

char buffer[80]; // Must be able to hold longest flash text string.

char* getFlashString(int i){
  strcpy_P(buffer, (char*)pgm_read_word(&(string_table[i])));
}
