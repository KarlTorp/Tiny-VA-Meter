# Tiny-VA-Meter
Compact Volt / Amp meter  using Arduino nano, a 128x64 OLED display and the INA219 high side current sensor.

Single button interface with short/long press.

Features:
- Display voltage and current, watt and mAh
- Calculates mAh with manual reset.
- Select INA126 ranges 32V/3.2A, 32V/1A or 16V/0.4A
- Select sampling rate 100, 200, 500 or 1000 ms.
- Enable/disable sensor sleep to lower leak current in sensor.
- Printout results on serial.
- Change settings with serial commands.
- Input detection with range feedback.
