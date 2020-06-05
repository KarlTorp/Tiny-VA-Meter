# Tiny-VA-Meter
Compact Volt / Amp meter  using Arduino nano, a 128x64 OLED display and the INA219 high side current sensor.

Single button interface with short/long press.

Features:
- Display voltage, current, watt and mAh
- Calculates mAh with manual reset.
- Select INA219 ranges 32V/3.2A, 32V/1A or 16V/0.4A
- Select sampling rate 100, 200, 500 or 1000 ms.
- Enable/disable sensor sleep to lower leak current in sensor.
- Settings are stored in EEPROM and reloaded on boot
- Print results on serial.
- Change settings with serial commands.
- Input detection with range feedback.

BOM:
- Arduino Nano
- INA219 sensor board (compact purple board)
- OLED 0.96" I2C 128X64 4-pin.
- TTP223 Capacitive Touch Switch
- Female Power Supply Jack Socket Mount
- Male Power Supply Jack
- Slide Switch 2 Position 6 Pin
- 5 pin male & female connector (optional)

Find guide and more here:
https://www.instructables.com/id/Tiny-VA-Meter-With-INA219/
