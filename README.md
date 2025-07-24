# Can Servo
An upgrade to 55g servo motors to make them more usable
This upgrade consists of removing the original mainboard + potentiomenter and upgrading them with the following features

This is still a work in progess
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
- [X] Velocity Control
- [X] PID parameters over CAN
- [X] Motor Sleep
- [X] "All respond" node id
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


<br>
<br>

## Build Instructions
### BOM
#### Parts From Servo
- Motor
- Metal Casing
- Gearbox
- 4 case screws
- 2 motor screws

#### Printed Parts
- Encoder Holder
- Encoder Cover
- Magnet Holder

#### Electronics
- 10K NTC Thermistor
- Populated Servo Driver PCB
- AS5600 Encoder

#### Additional Parts
- Magnet
- Bearing
- 6x JST 2.54 M connector (not pictured)
<img width="1711" height="2287" alt="image" src="https://github.com/user-attachments/assets/1f1d2018-6f41-4869-96e2-782ec6a991c0" />

1. Mod AS5600 to work in analog mode
<img width="2393" height="1010" alt="image" src="https://github.com/user-attachments/assets/ad12ed3d-5b38-4326-b0d7-a23d30802eb0" />
Remove R4
<br>
2. Insert Bearing into Encoder Cover
<img width="2192" height="1928" alt="image" src="https://github.com/user-attachments/assets/565e47b2-2e5f-4bd7-8c3d-054af7e22d87" />
<br>
3. Flatter Botton of Existing Gearbox
<img width="1775" height="1205" alt="image" src="https://github.com/user-attachments/assets/4ad51890-9868-472a-8e6f-20def62659db" />
<img width="1594" height="944" alt="image" src="https://github.com/user-attachments/assets/dbc23e6b-adb0-4224-9af9-92d3ca1f475f" />
<br>
4. Glue magnet to bottom of the Magnet Holder, insert into the sanded piece, add back the following gears and motor screws
<img width="970" height="1113" alt="image" src="https://github.com/user-attachments/assets/59e4e361-6c50-4fde-bfe1-a48c40417978" />
<br>
5. Install Encoder into Encoder Holder
<img width="2135" height="1237" alt="image" src="https://github.com/user-attachments/assets/de43dee4-8045-4f2f-ab78-aa89fcac8514" />

6. Solder wires to encoder through the bottom (only the 3 pin, not I2C), install motor into the printed piece
![PXL_20250722_003845700](https://github.com/user-attachments/assets/e946003c-0914-4e7a-a054-23b00d74ece1)
<br>
7. Solder one leg of NTC thermistor to VCC on the top of the encoder
<img width="1954" height="1707" alt="image" src="https://github.com/user-attachments/assets/2ecffa98-beb6-46f5-ba53-daa64a18ecc5" />
Feed thermistor leg up from the bottom
<br>
8. Install Encoder Cover
<img width="1746" height="1210" alt="image" src="https://github.com/user-attachments/assets/bb4f7428-5c87-4a13-8525-cbce6ac04521" />
<br>
9. Align motor screw holes to the bottom of the gearbox, screw in motor screws
<img width="1744" height="1774" alt="image" src="https://github.com/user-attachments/assets/a3d79bc9-aff0-4c2b-b096-0da1c6bb75f3" />
<br>
10. Assemble gearbox, screw in case screws, the old bottom of the servo is no longer used
<br>
11. Crimp 6 pin JST connector
<img width="1452" height="506" alt="image" src="https://github.com/user-attachments/assets/bf21b6b8-4036-411c-86f7-7638622db07f" />
From left to right:
encoder position output
the other leg of the thermistor
power to the encode
connection to 1/2 motor leads
connection to 1/2 motor leads
ground for the encoder
