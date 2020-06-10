// Using Arduino 1.8.10
#include <Arduino.h>
#include <Wire.h>
// Using U8g2 2.24.3
#include <U8g2lib.h>
// Using Adafruit INA219 1.0.5
#include <Adafruit_INA219.h>

#define ENABLE_TERMINAL 1 // Comment in to disable serial communication. Removes use of serial. Free 2.7KB(9%) Flash & 330B(16%) RAM. 
#define ENABLE_EEPROM_SETTINGS 1 // Comment in to disable eeprom settings. Free 420B(1%) Flash.
#define ENABLE_SETTINGS_OVERVIEW 1 // Comment in to disable mini settings overwiew. Removes use of one font. Free 2.1KB(7%) Flash & 58B(2%) RAM. 

#include "FlashMem.h"

#ifdef ENABLE_EEPROM_SETTINGS
#include <EEPROM.h>
#define EEPROM_SETTINGS_VALID 0x55 // Used to validate settings stored in EEPROM.
enum {
  EEPROM_VALIDATION_ADDR = 0,
  EEPROM_INA_RANGE_ADDR = 1,
  EEPROM_SENSOR_SLEEP_ADDR = 2,
  EEPROM_REFRESH_RATE_H_ADDR = 3,
  EEPROM_REFRESH_RATE_L_ADDR = 4
};
#endif

enum t_menu_pages{
  MENU_MAIN,
  MENU_VA,
  MENU_P,
  MENU_SETTINGS_ENTER,
  MENU_SETTINGS_RANGE,
  MENU_SETTINGS_REFRESH,
  MENU_SETTINGS_SLEEP
};

enum {
  INA219_RANGE_32V_3A = 0,
  INA219_RANGE_32V_1A = 1,
  INA219_RANGE_16V_400mA = 2
};

enum t_button_state {
  STATE_NONE = 0,
  STATE_PRESSED,
  STATE_PRESSED_LONG,
};

#define POWER_SELECT_PIN 6          // Indicator input for USB or input power
#define BUTTON_PIN 2
#define BUTTON_PRESSED HIGH         // Active state for button
#define USB_POWER_INPUT_STATE HIGH  // Input pin state for USB powered
#define LONG_PRESS_DURATION 1000    // ms.
#define SEC_PR_HOUR 3600

Adafruit_INA219 ina219;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA);

bool sensor_sleep = false;
bool serial_auto_send = true;
unsigned long currentMillis = 0;
unsigned long last_refresh = 0;
unsigned long refresh_rate = 200;
t_menu_pages current_menu = MENU_MAIN;
uint8_t current_ina_range = INA219_RANGE_32V_3A;
int power_select_known_state = 0;
unsigned long power_select_change_time = 0;
float mAh = 0;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage= 0;
float power_mw = 0;
t_button_state button_state;
unsigned long button_pressed_start = 0;

void setup(void) 
{
  // Configure pin BUTTON_PIN & POWER_SELECT_PIN as inputs and enable the internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_SELECT_PIN, INPUT_PULLUP);
  // Initialize OLED
  u8g2.begin();

  // Initializ INA219
  ina219.begin();
  
#ifdef ENABLE_TERMINAL
  // Initialize Serial
  Serial.begin(115200);
#endif

  load_eeprom_settings();
}

void loop(void) 
{
  currentMillis = millis();
  read_button();
  receive_serial();
  update_screen();
  check_power_source();
}

void load_eeprom_settings()
{
#ifdef ENABLE_EEPROM_SETTINGS
  if(EEPROM.read(EEPROM_VALIDATION_ADDR) == EEPROM_SETTINGS_VALID){
    set_INA_range(EEPROM.read(EEPROM_INA_RANGE_ADDR));
    sensor_sleep = EEPROM.read(EEPROM_SENSOR_SLEEP_ADDR) != 0;    
    refresh_rate = (unsigned long)EEPROM.read(EEPROM_REFRESH_RATE_H_ADDR) << 8;
    refresh_rate += (unsigned long)EEPROM.read(EEPROM_REFRESH_RATE_L_ADDR); 
  } else {
    Serial.println("No settings in EEPROM");
  }
#endif
}

void update_eeprom_settings()
{
#ifdef ENABLE_EEPROM_SETTINGS
  // Only write new values. Eeprom has a limited number of write operations.
      EEPROM.update(EEPROM_VALIDATION_ADDR, EEPROM_SETTINGS_VALID);
      EEPROM.update(EEPROM_INA_RANGE_ADDR, current_ina_range);
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

void set_INA_range(uint8_t range)
{
  current_ina_range = range;
  switch(range){
    case INA219_RANGE_16V_400mA:
      ina219.setCalibration_16V_400mA();
      break;
    case INA219_RANGE_32V_1A:
      ina219.setCalibration_32V_1A();
      break;
    case INA219_RANGE_32V_3A:
    default:
      current_ina_range = INA219_RANGE_32V_3A;
      ina219.setCalibration_32V_2A();
      break;
  }
}

void display_input_range()
{  
  u8g2.firstPage();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  do {
    u8g2.drawStr(0,32,"Input range:");
    if(power_select_known_state == USB_POWER_INPUT_STATE)  {
      u8g2.drawXBMP(46, 2, USB_icon_width, USB_icon_height, USB_icon_bits);
      u8g2.drawStr(0,56,"0-26V 3.2A");
    } else {
      u8g2.drawXBMP(30, 2, power_jack_icon_width, power_jack_icon_height, power_jack_icon_bits);
      u8g2.drawStr(0,56,"4-15V 3.2A");     
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

void display_settings()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(22,16,"Settings");

  if(power_select_known_state == USB_POWER_INPUT_STATE) {
    u8g2.drawXBMP(46, 22, USB_icon_width, USB_icon_height, USB_icon_bits);
  } else { 
    u8g2.drawXBMP(30, 22, power_jack_icon_width, power_jack_icon_height, power_jack_icon_bits);     
  }  
#ifdef ENABLE_SETTINGS_OVERVIEW
    String rate = "Refresh rate: " + String(refresh_rate) + " ms";
    String text = "Sensor range: ";
    if(current_ina_range == INA219_RANGE_32V_3A) {
      text += "26V & 3.2A";
    } else if(current_ina_range == INA219_RANGE_32V_1A) {
      text += "26V & 1A";
    } else { // INA219_RANGE_16V_400mA
      text += "16V & 0.4A";
    }

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0,46, text.c_str());
    if(sensor_sleep) {
      u8g2.drawStr(0,54, "Sensor sleep: Enabled");
    } else {
      u8g2.drawStr(0,54, "Sensor sleep: Disabled");
    }   
    u8g2.drawStr(0,62, rate.c_str());
#endif
  } while ( u8g2.nextPage() );
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
    current_menu = MENU_SETTINGS_RANGE;
    break;
  case MENU_SETTINGS_RANGE:
    if(current_ina_range == INA219_RANGE_32V_3A) {
      set_INA_range(INA219_RANGE_32V_1A);
    } else if(current_ina_range == INA219_RANGE_32V_1A) {
      set_INA_range(INA219_RANGE_16V_400mA);
    } else {
      set_INA_range(INA219_RANGE_32V_3A);
    }    
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
      ina219.powerSave(false);
    }
    break;
  default:
    current_menu = MENU_MAIN;
    break;
  }
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
  case MENU_SETTINGS_RANGE:
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

void read_button()
{
  const int buttonRead = digitalRead(BUTTON_PIN);
  
  switch(button_state) {
  case STATE_NONE:
    if(buttonRead == BUTTON_PRESSED) {
      button_state = STATE_PRESSED;
      button_pressed_start = currentMillis;
    }
    break;
  case STATE_PRESSED:
    if(buttonRead == BUTTON_PRESSED) {
      if(button_pressed_start + LONG_PRESS_DURATION < currentMillis) {
          button_state = STATE_PRESSED_LONG;
          long_press();
      }
    } else {
      button_state = STATE_NONE;
      short_press();
    }
    break;
  case STATE_PRESSED_LONG:
    if(buttonRead != BUTTON_PRESSED) {
      button_state = STATE_NONE;
    }
    break;
  }
}

void update_screen()
{
  if(last_refresh + refresh_rate <= currentMillis) {
    last_refresh += refresh_rate;
  
    if(sensor_sleep) {
      ina219.powerSave(false);
      delay(2); // Allow sensor boot < 1 ms.
    }
  
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    power_mw = ina219.getPower_mW();

    if(sensor_sleep) {
      ina219.powerSave(true);
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
      String volt = String(loadvoltage) + " V";
      String amp = String(current_mA) + " mA";
      print_two_lines(volt.c_str(), amp.c_str());
    }
      break;
    case MENU_P: // Display voltage, amps, watt and Ah
    {
      String volt = String(loadvoltage) + " V";
      String amp = String(current_mA) + " mA";
      String power = String(power_mw) + " mW";
      String ampHours = String(mAh) + " mAh";
      print_four_lines(volt.c_str(), amp.c_str(), power.c_str(), ampHours.c_str());
    }
      break;
    case MENU_SETTINGS_ENTER:
      display_settings();
      break;
    case MENU_SETTINGS_RANGE: // Display active INA219 sensor range
      if(current_ina_range == INA219_RANGE_32V_3A) {
        print_two_lines("Sensor range", "26V & 3.2A");
      } else if(current_ina_range == INA219_RANGE_32V_1A) {
        print_two_lines("Sensor range", "26V & 1A");
      } else { // INA219_RANGE_16V_400mA
        print_two_lines("Sensor range", "16V & 0.4A");
      }
      break;
    case MENU_SETTINGS_REFRESH: // Display active refresh rate
      {
        String rate = String(refresh_rate) + " ms";
        print_two_lines("Refresh rate", rate.c_str());
      }
      break;
    case MENU_SETTINGS_SLEEP: // Display active sensor sleep setting
      if(sensor_sleep) {
        print_two_lines("Sensor sleep", "Enabled");
      } else {
        print_two_lines("Sensor sleep", "Disabled");
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
    } else if(command.equals("range 0")) {
      set_INA_range(INA219_RANGE_32V_3A);
    } else if(command.equals("range 1")) {
      set_INA_range(INA219_RANGE_32V_1A);
    } else if(command.equals("range 2")) {
      set_INA_range(INA219_RANGE_16V_400mA);
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
      Serial.println(getFlashString(TXT_TERMINAL_HELP_RANGE));
      return;
    }
    Serial.println("OK");
    update_eeprom_settings();
  }
#endif
}
