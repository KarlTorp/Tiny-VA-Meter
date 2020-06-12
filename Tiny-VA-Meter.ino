// Using Arduino 1.8.10
#include <Arduino.h>
#include <Wire.h>
// Using U8g2 2.24.3
#include <U8g2lib.h>
// Using ArduinoINA219 fork from KarlTorp GitHub
#include <INA219.h>

#define ENABLE_TERMINAL 1 // Comment in to disable serial communication. Removes use of serial. Free 2.7KB(9%) Flash & 330B(16%) RAM. 
#define ENABLE_EEPROM_SETTINGS 1 // Comment in to disable eeprom settings. Free 420B(1%) Flash.
#define ENABLE_SETTINGS_OVERVIEW 1 // Comment in to disable mini settings overwiew. Removes use of one font. Free 2.1KB(7%) Flash & 58B(2%) RAM. 

#include "FlashMem.h"
#include "button.hpp"

#ifdef ENABLE_EEPROM_SETTINGS
#include <EEPROM.h>
#define EEPROM_SETTINGS_VALID 0x56 // Used to validate settings stored in EEPROM.
enum {
  EEPROM_VALIDATION_ADDR = 0,
  EEPROM_INA_RANGE_V_ADDR,
  EEPROM_INA_RANGE_I_ADDR,
  EEPROM_INA_AVERAGING_ADDR,
  EEPROM_SENSOR_SLEEP_ADDR,
  EEPROM_REFRESH_RATE_H_ADDR,
  EEPROM_REFRESH_RATE_L_ADDR,
};
#endif

enum {
  MENU_MAIN,
  MENU_VA,
  MENU_P,
  MENU_SETTINGS_ENTER,
  MENU_SETTINGS_RANGE_V,
  MENU_SETTINGS_RANGE_I,
  MENU_SETTINGS_AVERAGING,
  MENU_SETTINGS_REFRESH,
  MENU_SETTINGS_SLEEP
};

struct Ina_config_settings {
  INA219::t_range range;
  INA219::t_gain gain;
  INA219::t_adc bus_adc;
  INA219::t_adc shunt_adc;
  INA219::t_mode mode;
};

struct Ina_calibration_settings {
  float v_shunt_max;
  float v_bus_max;
  float i_bus_max_expected;
};
                

#define POWER_SELECT_PIN 6        // Indicator input for USB or input power
#define BUTTON_PIN 2
#define BUTTON_PRESSED_STATE HIGH // Active state for button
#define USB_POWER_INPUT_STATE HIGH // Input pin state for USB powered
#define LONG_PRESS_DURATION 1000  // ms.
#define DEBOUNCE_DURATION 50 // ms.
#define SEC_PR_HOUR 3600
#define R_SHUNT 0.1

INA219 ina219;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA);
BUTTON button(BUTTON_PIN, true, BUTTON_PRESSED_STATE, LONG_PRESS_DURATION, DEBOUNCE_DURATION);

Ina_config_settings ina_config;
Ina_calibration_settings ina_calib;

bool sensor_sleep = false;
bool serial_auto_send = true;
bool wait_for_lp_release = false;
unsigned long currentMillis = 0;
unsigned long last_refresh = 0;
unsigned long refresh_rate = 200;
uint8_t current_menu = MENU_MAIN;
float mAh = 0;
int power_select_known_state = 0;
unsigned long power_select_change_time = 0;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage= 0;
float power_mw = 0;

void setup(void) 
{
  // Configure pin POWER_SELECT_PIN as inputs and enable the internal pull-up resistor
  pinMode(POWER_SELECT_PIN, INPUT_PULLUP);
  // Initialize OLED
  u8g2.begin();
  
#ifdef ENABLE_TERMINAL
  // Initialize Serial
  Serial.begin(115200);
#endif
  // Initializ INA219
  ina219.begin();
  load_settings();
  update_ina_config();
}

void loop(void) 
{
  currentMillis = millis();
  read_button();
  receive_serial();
  update_screen();
  check_power_source();
}

void read_button()
{
  switch(button.updateState()){
    case BUTTON::STATE_RELEASED_SP:
      short_press();
      break;
    case BUTTON::STATE_PRESSED_LP:
      if(!wait_for_lp_release) {
        long_press();
        wait_for_lp_release = true;
      }
      break; 
    case BUTTON::STATE_RELEASED_LP:
       wait_for_lp_release = false;
       break;
     default:
       break;
  }
}

void load_settings()
{
  ina_config.mode = INA219::CONT_SH_BUS;
#ifdef ENABLE_EEPROM_SETTINGS
  if(EEPROM.read(EEPROM_VALIDATION_ADDR) == EEPROM_SETTINGS_VALID){
    set_voltage_range((INA219::t_range)EEPROM.read(EEPROM_INA_RANGE_V_ADDR));
    set_current_range((INA219::t_gain)EEPROM.read(EEPROM_INA_RANGE_I_ADDR));
    ina_config.shunt_adc = (INA219::t_adc)EEPROM.read(EEPROM_INA_AVERAGING_ADDR);
    ina_config.bus_adc = ina_config.shunt_adc;
    sensor_sleep = EEPROM.read(EEPROM_SENSOR_SLEEP_ADDR) != 0;    
    refresh_rate = (unsigned long)EEPROM.read(EEPROM_REFRESH_RATE_H_ADDR) << 8;
    refresh_rate += (unsigned long)EEPROM.read(EEPROM_REFRESH_RATE_L_ADDR); 
    return;
  } 
#endif
  ina_config.bus_adc = INA219::ADC_12BIT;
  ina_config.shunt_adc = INA219::ADC_12BIT;
  set_voltage_range(INA219::RANGE_32V);
  set_current_range(INA219::GAIN_8_320MV);
}

void update_eeprom_settings()
{
#ifdef ENABLE_EEPROM_SETTINGS
  // Only write new values. Eeprom has a limited number of write operations.
  EEPROM.update(EEPROM_VALIDATION_ADDR, EEPROM_SETTINGS_VALID);
  EEPROM.update(EEPROM_INA_RANGE_V_ADDR, ina_config.range);  
  EEPROM.update(EEPROM_INA_RANGE_I_ADDR, ina_config.gain);
  EEPROM.update(EEPROM_INA_AVERAGING_ADDR, ina_config.shunt_adc);
  EEPROM.update(EEPROM_SENSOR_SLEEP_ADDR, (uint8_t)sensor_sleep);
  EEPROM.update(EEPROM_REFRESH_RATE_H_ADDR, (refresh_rate >> 8) & 0xFF);
  EEPROM.update(EEPROM_REFRESH_RATE_L_ADDR, refresh_rate & 0xFF);
#endif
}

void check_power_source() 
{
  if(power_select_known_state != digitalRead(POWER_SELECT_PIN)){
    current_menu = MENU_MAIN;
    power_select_change_time = currentMillis;
  }
}

void update_ina_config()
{
  ina219.calibrate(R_SHUNT, ina_calib.v_shunt_max, ina_calib.v_bus_max, ina_calib.i_bus_max_expected);
  ina219.configure(ina_config.range, ina_config.gain, 
  ina_config.bus_adc, ina_config.shunt_adc, ina_config.mode );
}

void display_input_range()
{  
  u8g2.firstPage();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  do {
    u8g2.drawStr(0,32, getFlashString(TXT_INPUT_RANGE));
    if(power_select_known_state == USB_POWER_INPUT_STATE)  {
      u8g2.drawXBMP(46, 2, USB_icon_width, USB_icon_height, USB_icon_bits);
      u8g2.drawStr(0,56,getFlashString(TXT_JACK_RANGE));
    } else {
      u8g2.drawXBMP(30, 2, power_jack_icon_width, power_jack_icon_height, power_jack_icon_bits);
      u8g2.drawStr(0,56,getFlashString(TXT_USB_RANGE));     
    }
  } while ( u8g2.nextPage() );
}

void print_four_lines(const char* line1, const char* line2, const char* line3, const char* line4)
{
  u8g2.firstPage();
  u8g2.setFont(u8g2_font_ncenB12_tr);
  do {
    u8g2.drawStr(0,15,line1);
    u8g2.drawStr(0,31,line2);
    u8g2.drawStr(0,47,line3);
    u8g2.drawStr(0,63,line4);
  } while ( u8g2.nextPage() );
}

void print_two_lines(const char* line1, const char* line2)
{
  u8g2.firstPage();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  do {
    u8g2.drawStr(0,24,line1);
    u8g2.drawStr(0,52,line2);
  } while ( u8g2.nextPage() );
}

String parseFloatTextWithUnit(float value, uint8_t decimals, const char* unit)
{
  return String(value, decimals) + unit;
}

String parseTextFloatAndUnit(const char* text, float value, uint8_t decimals, const char* unit)
{
  return text + String(value, decimals) + unit;
}

void display_settings()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(22, 16, getFlashString(TXT_SETTINGS));

  if(power_select_known_state == USB_POWER_INPUT_STATE) {
    u8g2.drawXBMP(46, 22, USB_icon_width, USB_icon_height, USB_icon_bits);
  } else { 
    u8g2.drawXBMP(30, 22, power_jack_icon_width, power_jack_icon_height, power_jack_icon_bits);     
  }  
#ifdef ENABLE_SETTINGS_OVERVIEW
    String text = "Sensor range: " + String(ina_calib.v_bus_max,0) + "V & " + String(ina_calib.i_bus_max_expected,1) + "A";
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0,46, text.c_str());
    String sleep_text = getFlashString(TEXT_SENSOR_SLEEP);
    if(sensor_sleep) {
      sleep_text += ": Enabled";
    } else {
      sleep_text += ": Disabled";
    }   
    u8g2.drawStr(0,54, sleep_text.c_str());
    u8g2.drawStr(0,62, parseTextFloatAndUnit("Refresh rate: ", refresh_rate, 0, " ms").c_str());
#endif
  } while ( u8g2.nextPage() );
}

void set_voltage_range(INA219::t_range range)
{
  ina_config.range = range;
  if(range == INA219::RANGE_32V) {
    ina_calib.v_bus_max = 32;
  } else {
    ina_calib.v_bus_max = 16;
  }
}

void set_current_range(INA219::t_gain gain)
{
  ina_config.gain = gain;
    if(ina_config.gain == INA219::GAIN_1_40MV)  {
      ina_calib.v_shunt_max = 0.04;
      ina_calib.i_bus_max_expected = 0.4;
    } else if(ina_config.gain == INA219::GAIN_2_80MV)  {
      ina_calib.v_shunt_max = 0.08;
      ina_calib.i_bus_max_expected = 0.8;
    } else if(ina_config.gain == INA219::GAIN_4_160MV)  {
      ina_calib.v_shunt_max = 0.16;
      ina_calib.i_bus_max_expected = 1.6;
    } else {
      ina_calib.v_shunt_max = 0.320;
      ina_calib.i_bus_max_expected = 3.2;
    }
}

void long_press()
{
  switch(current_menu) {
  case MENU_MAIN:
    current_menu = MENU_VA;
    break;
  case MENU_VA:
    break;
  case MENU_P:
    mAh = 0;
    break;
  case MENU_SETTINGS_ENTER:
    current_menu = MENU_SETTINGS_RANGE_V;
    break;
  case MENU_SETTINGS_RANGE_V:
    if(ina_config.range == INA219::RANGE_16V) {
      set_voltage_range(INA219::RANGE_32V);
    } else {
      set_voltage_range(INA219::RANGE_16V);
    }
    break;
  case MENU_SETTINGS_RANGE_I:
    if(ina_config.gain == INA219::GAIN_1_40MV)  {
      set_current_range(INA219::GAIN_2_80MV);
    } else if(ina_config.gain == INA219::GAIN_2_80MV)  {
      set_current_range(INA219::GAIN_4_160MV);
    } else if(ina_config.gain == INA219::GAIN_4_160MV)  {
      set_current_range(INA219::GAIN_8_320MV);
    } else {
      set_current_range(INA219::GAIN_1_40MV);
    } 
    break;
  case MENU_SETTINGS_AVERAGING:
    if(ina_config.shunt_adc == INA219::ADC_128SAMP) {
      ina_config.shunt_adc = INA219::ADC_12BIT;
    } else if(ina_config.shunt_adc == INA219::ADC_12BIT) {
      ina_config.shunt_adc = INA219::ADC_2SAMP;
    } else {
      uint8_t current = ina_config.shunt_adc + 1;
      ina_config.shunt_adc = (INA219::t_adc)current;
    }
    ina_config.bus_adc = ina_config.shunt_adc;
    break;
  case MENU_SETTINGS_REFRESH:
    if(refresh_rate == 100) {
      refresh_rate = 200;
    } else if(refresh_rate == 200) {
      refresh_rate = 500;
    } else if(refresh_rate == 500) {
      refresh_rate = 1000;
    } else {
      refresh_rate = 100;
    } 
      break;
  case MENU_SETTINGS_SLEEP:
    sensor_sleep = !sensor_sleep;
    if(!sensor_sleep){
      ina_config.mode = INA219::CONT_SH_BUS;
    }
    break;
  default:
    current_menu = MENU_MAIN;
    break;
  }
  update_ina_config();
  update_eeprom_settings();
}
void short_press()
{
  switch(current_menu) {
  case MENU_MAIN:
    current_menu = MENU_VA;
    break;
  case MENU_VA:
    current_menu = MENU_P;
    break;
  case MENU_P:
    current_menu = MENU_SETTINGS_ENTER;
    break;
  case MENU_SETTINGS_ENTER:
    current_menu = MENU_VA;
    break;
  case MENU_SETTINGS_RANGE_V:
    current_menu = MENU_SETTINGS_RANGE_I;
    break;
  case MENU_SETTINGS_RANGE_I:
    current_menu = MENU_SETTINGS_AVERAGING;
    break;
  case MENU_SETTINGS_AVERAGING:
    current_menu = MENU_SETTINGS_REFRESH;
    break;
  case MENU_SETTINGS_REFRESH:
    current_menu = MENU_SETTINGS_SLEEP;
    break;
  case MENU_SETTINGS_SLEEP:
    current_menu = MENU_VA;
    break;
  default:
    current_menu = MENU_MAIN;
    break;
  }
}

void update_screen()
{
  if(last_refresh + refresh_rate <= currentMillis) {
    last_refresh += refresh_rate;
  
    if(sensor_sleep) {
      ina_config.mode = INA219::CONT_SH_BUS;
      update_ina_config();
      delay(2); // Allow sensor boot < 1 ms.
    }
  
    shuntvoltage = ina219.shuntVoltage();
    busvoltage = ina219.busVoltage();
    current_mA = ina219.shuntCurrent()*1000;
    loadvoltage = busvoltage + shuntvoltage;
    power_mw = ina219.busPower()*1000;

    if(sensor_sleep) {
      ina_config.mode = INA219::PWR_DOWN;
      update_ina_config();
    }

    const float samlpe_rate = (float)refresh_rate;
    mAh += current_mA / ((1000/samlpe_rate) * SEC_PR_HOUR);
    if(serial_auto_send) {
      transmit_serial();
    }
  }
  
  switch(current_menu) {
    case MENU_MAIN:
      power_select_known_state = digitalRead(POWER_SELECT_PIN);
      display_input_range();
      if(currentMillis >= power_select_change_time + 3000) {
        short_press();
      }
      break;
    case MENU_VA: // Display voltage and amps
    {
      print_two_lines(parseFloatTextWithUnit(loadvoltage, 2, " V").c_str(), parseFloatTextWithUnit(current_mA, 1, " mA").c_str());
    }
      break;
    case MENU_P: // Display voltage, amps, watt and Ah
    {
      print_four_lines(parseFloatTextWithUnit(loadvoltage, 2, " V").c_str(), parseFloatTextWithUnit(current_mA, 1, " mA").c_str(), parseFloatTextWithUnit(power_mw, 2, " mV").c_str(), parseFloatTextWithUnit(mAh, 1, " mAh").c_str());
    }
      break;
    case MENU_SETTINGS_ENTER:
      display_settings();
      break;
    case MENU_SETTINGS_RANGE_V:
    {
       print_two_lines("Range Volt", parseFloatTextWithUnit(ina_calib.v_bus_max, 0, " V").c_str());
    }   
       break;
    case MENU_SETTINGS_RANGE_I:
    {
       print_two_lines("Range Amp", parseFloatTextWithUnit(ina_calib.i_bus_max_expected, 1, " A").c_str());
    }   
      break;
    case MENU_SETTINGS_AVERAGING:
    {
      String number;
      if(ina_config.shunt_adc < INA219::ADC_2SAMP) {
        number = "Off";
      } else {
        uint8_t samples = 1 << ((uint8_t)ina_config.shunt_adc - 8);
        number = parseFloatTextWithUnit(samples, 0, " sampels");
      }
      print_two_lines("Averaging", number.c_str());
    } 
      break;
    case MENU_SETTINGS_REFRESH: // Display active refresh rate
      {
        print_two_lines("Refresh rate", parseFloatTextWithUnit(refresh_rate, 0, " ms").c_str());
      }
      break;
    case MENU_SETTINGS_SLEEP: // Display active sensor sleep setting
      if(sensor_sleep) {
        print_two_lines(getFlashString(TEXT_SENSOR_SLEEP), "Enabled");
      } else {
        print_two_lines(getFlashString(TEXT_SENSOR_SLEEP), "Disabled");
      }
      break;
    default:
      current_menu = MENU_MAIN;
      break;
  }
}
void transmit_serial()
{
#ifdef ENABLE_TERMINAL
    Serial.print("Millis:        "); Serial.print(currentMillis); Serial.println(" ms");
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mw); Serial.println(" mW");
    Serial.print("Comsumption:   "); Serial.print(mAh); Serial.println(" mAh");
    Serial.println("");
#endif
}


void receive_serial()
{
#ifdef ENABLE_TERMINAL
  String command;
  if(Serial.available()){
    command = Serial.readStringUntil('\n');   
    if(command.equals("reset")){
      mAh = 0;
    } else if(command.equals("sleep on")) {
      sensor_sleep = true;
    } else if(command.equals("sleep off")) {
      sensor_sleep = false;
    } else if(command.equals("refresh 100")) {
      refresh_rate = 100;
    } else if(command.equals("refresh 200")) {
      refresh_rate = 200;
    } else if(command.equals("refresh 500")) {
      refresh_rate = 500;
    } else if(command.equals("refresh 1000")) {
      refresh_rate = 1000;
    } else if(command.equals("volt 16")) {
      set_voltage_range(INA219::RANGE_16V);
    } else if(command.equals("volt 32")) {
      set_voltage_range(INA219::RANGE_32V);
    } else if(command.equals("gain 1")) {
      set_current_range(INA219::GAIN_1_40MV);
    } else if(command.equals("gain 2")) {
      set_current_range(INA219::GAIN_2_80MV);
    } else if(command.equals("gain 4")) {
      set_current_range(INA219::GAIN_4_160MV);
    } else if(command.equals("gain 8")) {
      set_current_range(INA219::GAIN_8_320MV);
    } else if(command.equals("avg 0")) {
      ina_config.shunt_adc = INA219::ADC_12BIT;
    } else if(command.equals("avg 2")) {
      ina_config.shunt_adc = INA219::ADC_2SAMP;
    } else if(command.equals("avg 4")) {
      ina_config.shunt_adc = INA219::ADC_4SAMP;
    } else if(command.equals("avg 8")) {
      ina_config.shunt_adc = INA219::ADC_8SAMP;
    } else if(command.equals("avg 16")) {
      ina_config.shunt_adc = INA219::ADC_16SAMP;
    } else if(command.equals("avg 32")) {
      ina_config.shunt_adc = INA219::ADC_32SAMP;
    } else if(command.equals("avg 64")) {
      ina_config.shunt_adc = INA219::ADC_64SAMP;
    } else if(command.equals("avg 128")) {
      ina_config.shunt_adc = INA219::ADC_128SAMP;
    } else if(command.equals("log off")) {
      serial_auto_send = false;
    } else if(command.equals("log on")) {
      serial_auto_send = true;
    } else if(command.equals("read")) {
      transmit_serial();
    } else {
      Serial.println(getFlashString(TXT_TERMINAL_HELP_HEAD));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_RESET));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_READ));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_LOG));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_SLEEP));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_REFRESH));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_RANGE_V));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_RANGE_I));
      Serial.println(getFlashString(TXT_TERMINAL_HELP_AVG));
      return;
    }
    Serial.println("OK");
    update_ina_config();
    update_eeprom_settings();
  }
#endif
}
