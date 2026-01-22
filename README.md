# Pavement-Sonnet-Firmware

## ESP32 Sensor Pinout List
### Power Control
|ESP Pin|Device Pin|Description|
|-|-|-|
|GPIO 25|200Ohm->I-MOSFET 2|Power for 3.3V Device|
|GPIO 26|200Ohm->I-MOSFET 1|Power for 5V Device|
|GPIO 27|VCC-Sound Sensor|Power for sound sensor|
### Sensor Pins
|ESP Pin|Device Pin|Description|
|-|-|-|
|GPIO 16|TX-GPS||
|GPIO 17|RX-GPS||
|GPIO 32|SIG-Temp Sensor|Digital|
|GPIO 34|MQ135 AOUT|Caution: UnderVolt the signal to 3.3V|
|GPIO 35|SIG-Sound|Analog|
|GPIO 14|Int-IMU|Accelrameter WakeUp|

### Sensor Pins
