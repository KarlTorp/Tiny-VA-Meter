// Using Arduino 1.8.5
#include <Arduino.h>
#include <Wire.h>
// Using U8g2 2.24.3
#include <U8g2lib.h>
// Using Adafruit INA219 1.0.5
#include <Adafruit_INA219.h>

#define SEC_PR_HOUR 3600

#define MENU_MAIN 0
#define MENU_VA 1
#define MENU_P 2
#define MENU_SETTINGS_ENTER 3
#define MENU_SETTINGS_RANGE 4
#define MENU_SETTINGS_REFRESH 5
#define MENU_SETTINGS_SLEEP 6

enum {
  INA219_RANGE_32V_3A,
  INA219_RANGE_32V_1A,
  INA219_RANGE_16V_400mA
};

#define INA219_RANGE_32V_3A 0
#define INA219_RANGE_32V_1A 1
#define INA219_RANGE_16V_400mA 2

const int BUTTON_PIN = 2;     // the number of the pushbutton pin
const int LED_PIN =  13;      // the number of the LED pin
const unsigned long LONG_PRESS_DURATION = 1000;

Adafruit_INA219 ina219;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, SCL, SDA);

bool button_pressed = false;
bool wait_for_button_release = false;
bool sensor_sleep = false;
unsigned long button_pressed_start = 0;
unsigned long currentMillis = 0;
unsigned long last_refresh = 0;
unsigned long refresh_rate = 200;
uint8_t current_menu = MENU_MAIN;
uint8_t current_ina_range = INA219_RANGE_32V_3A;
float mAh = 0;


void print_two_lines(const char* line1, const char* line2)
{
  u8g2.clearBuffer();          // clear the internal memory  
  u8g2.setCursor(0,14);
  u8g2.print(line1);
  u8g2.setCursor(0,32);
  u8g2.print(line2);  
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void print_one_line(const char* line1)
{
  u8g2.clearBuffer();          // clear the internal memory  
  u8g2.setCursor(0,14);
  u8g2.print(line1);
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void setup(void) {
  // Initialize the LED_PIN as an output:
  pinMode(LED_PIN, OUTPUT);
  // Configure pin BUTTON_PIN as an input and enable the internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // Initialize OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB14_tr); // choose a suitable font
  
  // Initialize Serial
  Serial.begin(115200);

  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
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
      current_ina_range = INA219_RANGE_32V_1A;
      ina219.setCalibration_32V_1A();
    } else if(current_ina_range == INA219_RANGE_32V_1A) {
      current_ina_range = INA219_RANGE_16V_400mA;
      ina219.setCalibration_16V_400mA();
    } else {
      current_ina_range = INA219_RANGE_32V_3A;
      ina219.setCalibration_32V_2A();
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
  const int buttonState = digitalRead(BUTTON_PIN);
  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if(buttonState == LOW) {
    digitalWrite(LED_PIN, HIGH);
    if(button_pressed == true) {
      if(!wait_for_button_release && (button_pressed_start + LONG_PRESS_DURATION < currentMillis)) {
        wait_for_button_release = true;
        long_press();
      }
    } else {
      button_pressed_start = currentMillis;
      button_pressed = true;
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    if(button_pressed == true)
    {
      if(!wait_for_button_release){
        short_press();
      }
    }
    wait_for_button_release = false;
    button_pressed = false;
  }
}

void update_screen()
{
  if(sensor_sleep) {
    ina219.powerSave(false);
    delay(2); // Allow sensor boot < 1 ms.
  }
  
  const float shuntvoltage = ina219.getShuntVoltage_mV();
  const float busvoltage = ina219.getBusVoltage_V();
  const float current_mA = ina219.getCurrent_mA();
  const float loadvoltage = busvoltage + (shuntvoltage / 1000);
  const float power_mw = ina219.getPower_mW();

  if(sensor_sleep) {
    ina219.powerSave(true);
  }

  const float samlpe_rate = (float)refresh_rate;

  mAh += current_mA / ((1000/samlpe_rate) * SEC_PR_HOUR);
  
  switch(current_menu) {
    case MENU_MAIN:
      print_two_lines("Input range:", "0-32V 0-3.2A");
      if(currentMillis >= 3000) {
        current_menu = MENU_VA;
      }
      break;
    case MENU_VA:
    {
      String volt = String(loadvoltage) + "/" + String(busvoltage) + " V";
      String amp = String(current_mA) + " mA";
      print_two_lines(amp.c_str(), volt.c_str());
    }
      break;
    case MENU_P:
    {
      String power = String(power_mw) + " mW";
      String ampHours = String(mAh) + " mAh";
      print_two_lines(power.c_str(), ampHours.c_str());
    }
      break;
    case MENU_SETTINGS_ENTER:
      print_one_line("Settings");
      break;
    case MENU_SETTINGS_RANGE:  
      if(current_ina_range == INA219_RANGE_32V_3A) {
        print_two_lines("Active range", "32V & 3.2A");
      } else if(current_ina_range == INA219_RANGE_32V_1A) {
        print_two_lines("Active range", "32V & 1A");
      } else { // INA219_RANGE_16V_400mA
        print_two_lines("Active range", "16V & 0.4A");
      }
      break;
    case MENU_SETTINGS_REFRESH:
    {
      String rate = String(refresh_rate) + " ms";
      print_two_lines("Refresh rate", rate.c_str());
    }
      break;
    case MENU_SETTINGS_SLEEP:
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
  

  Serial.print("Millis:        "); Serial.print(currentMillis); Serial.println(" ms");
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mw); Serial.println(" mW");
  Serial.print("Comsumption:   "); Serial.print(mAh); Serial.println(" mAh");
  Serial.println("");
}

void receive_serial()
{
  String command;
  if(Serial.available()){
    command = Serial.readStringUntil('\n');
     
    if(command.equals("reset")){
      mAh = 0;
      Serial.println("mAh reset");
    } else if(command.equals("sleep on")) {
      sensor_sleep = true;
      Serial.println("sleep on");
    } else if(command.equals("sleep off")) {
      sensor_sleep = false;
      Serial.println("sleep off");
    } else if(command.equals("refresh 100")) {
      refresh_rate = 100;
      Serial.println("refresh rate 100");
    } else if(command.equals("refresh 200")) {
      refresh_rate = 200;
      Serial.println("refresh rate 200");
    } else if(command.equals("refresh 500")) {
      refresh_rate = 500;
      Serial.println("refresh rate 500");
    } else if(command.equals("refresh 1000")) {
      refresh_rate = 1000;
      Serial.println("refresh rate 1000");
    } else {
        Serial.println("Invalid command");
    }
  }
}


void loop(void) {
  currentMillis = millis();
  read_button();
  receive_serial();
  if(last_refresh + refresh_rate <= currentMillis){
    last_refresh += refresh_rate;
    update_screen();
  }
}

