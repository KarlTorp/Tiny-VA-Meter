// Using Arduino 1.8.5
#include <Arduino.h>
#include <Wire.h>
// Using U8g2 2.24.3
#include <U8g2lib.h>
// Using Adafruit INA219 1.0.2
// TODO update or change.
#include <Adafruit_INA219.h>

const int BUTTON_PIN = 2;     // the number of the pushbutton pin
const int LED_PIN =  13;      // the number of the LED pin

Adafruit_INA219 ina219;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

void setup(void) {
  // Initialize the LED_PIN as an output:
  pinMode(LED_PIN, OUTPUT);
  // Configure pin BUTTON_PIN as an input and enable the internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // Initialize OLED
  u8g2.begin();
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void read_button()
{
  int buttonState = digitalRead(BUTTON_PIN);

  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if (buttonState == LOW) {
    // turn LED on:
    digitalWrite(LED_PIN, HIGH);
  } else {
    // turn LED off:
    digitalWrite(LED_PIN, LOW);
  }
}

void update_screen()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
  
  u8g2.clearBuffer();          // clear the internal memory
  
  u8g2.setFont(u8g2_font_ncenB14_tr); // choose a suitable font
  u8g2.setCursor(0,14);
  u8g2.print(current_mA);
  u8g2.print(" mA");
  u8g2.setCursor(0,32);
  u8g2.print(loadvoltage);
  u8g2.print(" V");
  
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void loop(void) {
  read_button();
  update_screen();
  delay(100);  
}
