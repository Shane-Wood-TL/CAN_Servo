# Can Servo
An upgrade to 55g servo motors to make them more usable

This is still a work in progress, check out the [wiki](https://github.com/Shane-Wood-TL/CAN_Servo/wiki) for more detailed information.

This upgrade consists of removing the original mainboard + potentiomenter and upgrading them with the following features:
## Features
- [x] Temperature and current sensing
- [x] Magnetic encoder for positioning
- [x] Control over CAN (TWAI) bus
- [x] Daisy chainable motors
- [x] Onboard ESP32-C6, running FreeRTOS (ESP-IDF)
- [x] Setable CAN ID
- [x] Able to communicate sensor information back to other CAN nodes
- [x] Setting RGB color
- [x] Position Control
- [x] PID
- [X] Variable motor speed
- [X] Limiting Current / Temperature
- [X] Using RGB for status
- [X] PID parameters over CAN
- [X] Motor Sleep
- [X] "All respond" node id
- [X] Reboot Command
- [X] Add RXSDO and TXSDO for getting and setting values without a specified command
- [X] Add MCU temperature TXSDO
- [X] Add MCU MAC address TXSDO
- [X] Basic Velocity Control
<br>

### Planned Features
- [ ] Proper Velocity Control (Change Velocity to RPM, rather than just "speed")

<br>

### Additional Work:
- [ ] Upload CAD
- [ ] Upload PCB
- [ ] Upload Schematic
- [ ] PDF Build Guide
<br>

# PCB BOM
1 x ESP32-C6-Zero (Waveshare) <br>
1 x MP6550 Driver Carrier (Pololu)<br>
1 x 10K resistor<br>
1 x TJA1050, CAN Transceiver<br>
1 x "Mini 360" Buck converter, MP2307<br>
2 x 4p f 2.54 JST<br>
1 x 6p f 2.54 JST<br>
