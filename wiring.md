# Wiring

## Arduino Nano

| Nano pin | Connects to |
|---|---|
| 5V | LM2596 OUT+, TB6612 VCC, MPU6050 VCC, 100nF cap (with a 100uF electrolytic in parallel) |
| GND | LM2596 OUT-, TB6612 GND, MPU6050 GND, MPU6050 AD0, other cap leg |
| A4 | MPU6050 SDA |
| A5 | MPU6050 SCL |
| D2 | MPU6050 INT (not used by the current sketches) |
| D5 | TB6612 PWMA |
| D6 | TB6612 AIN2 |
| D7 | TB6612 AIN1 |
| D8 | TB6612 STBY |
| D9 | TB6612 BIN1 |
| D10 | TB6612 BIN2 |
| D11 | TB6612 PWMB |

## TB6612FNG

| Pin | Connects to |
|---|---|
| VM | Battery positive (after fuse and switch), 470uF / 25V cap to GND |
| VCC | Nano 5V |
| GND | Common ground |
| AO1 / AO2 | Motor A |
| BO1 / BO2 | Motor B |

## Power

- Battery + -> 5A fuse -> switch -> LM2596 IN+ and TB6612 VM
- Battery - -> LM2596 IN- and common ground
- LM2596 output set to 5.0V with a multimeter before connecting anything
- Never connect USB and the LM2596 output to the Nano at the same time
