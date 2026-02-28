<img width="2880" height="1824" alt="圖片" src="https://github.com/user-attachments/assets/9e46fac2-5c8f-4f88-acbd-4c334b8e0413" /># Pavement-Sonnet-Pinout

The following is the list of sensors and components utilized in this project, if you want to reproduce the project, please connect the components like this:

### 2.1 ESP32 Development Board

* **VSCode Plugin:** PlatformIO
* **Libraries:** `Arduino.h`
* **Pinout Table:**

| Pin / GPIO | Connection Destination |
| --- | --- |
| **GPIO 16** | GPS TX |
| **GPIO 17** | GPS RX |
| **GPIO 18** | LCD TX |
| **GPIO 19** | LCD RX |
| **GPIO 32** | TEMP SIG |
| **GPIO 34** | MQ135 AOUT |
| **GPIO 35** | SIG-Sound |
| **GPIO 14** | MPU INT |
| **GPIO 21** | MPU SDA |
| **GPIO 22** | MPU SCL |
| **GPIO 25** | I-MOSFET 2 |
| **GPIO 26** | I-MOSFET 1 |
| **GPIO 27** | VCC-Sound Sensor |
| **GND** | Common GND |
| **3V3** | Common 3V3 |
| **5V** | Common 5V |

---

### 2.2 MPU6050 MPU (Accelerometer)

* **Libraries:** `Adafruit_MPU6050.h`, `Adafruit_Sensor.h`
* **Pinout Table:**

| MPU Pin | Connection Destination |
| --- | --- |
| **MPU INT** | GPIO 14 |
| **MPU SDA** | GPIO 21 |
| **MPU SCL** | GPIO 22 |
| **GND** | Common GND |
| **3V3** | Common 3V3 |

---

### 2.3 NEO-6M v2 GPS

* **Libraries:** `TinyGPS.h`, `HardwareSerial.h`
* **Pinout Table:**

| GPS Pin | Connection Destination |
| --- | --- |
| **TX** | GPIO 16 |
| **RX** | GPIO 17 |
| **GND** | **D-MOSFET 2** (Ground Switching) |
| **3V3** | Common 3V3 |

---

### 2.4 LM358 Sound Sensor

* **Libraries:** `Arduino.h` (Analog Input)
* **Pinout Table:**

| Sensor Pin | Connection Destination |
| --- | --- |
| **SIG** | GPIO 35 |
| **VCC** | GPIO 27 |
| **GND** | Common GND |

---

### 2.5 DS18B20 Temperature Sensor

* **Libraries:** `OneWire.h`, `DallasTemperature.h`
* **Pinout Table:**

| Sensor Pin | Connection Destination |
| --- | --- |
| **SIG** | GPIO 32 |
| **VCC** | Common 3V3 |
| **GND** | **D-MOSFET 2** (Ground Switching) |

---

### 2.6 MQ135 Air Quality Sensor

* **Libraries:** `Arduino.h` (Analog Input)
* **Pinout Table:**

| Sensor Pin | Connection Destination |
| --- | --- |
| **SIG** | GPIO 34 (down volt) |
| **VCC** | Common 5V |
| **GND** | **D-MOSFET 1** (Ground Switching) |

---

### 2.7 1100 mAh Li-Ion Battery

| Terminal | Connection Destination |
| --- | --- |
| **+** | BAT + |
| **-** | BAT - |

---

### 2.8 MCP73833 LIPO/Li-Ion Charger Controller

| Port | Connection Destination |
| --- | --- |
| **IN +** | Solar + |
| **IN -** | Solar - |
| **LOAD +** | Step Up IN + |
| **LOAD -** | Step Up IN - |
| **BAT +** | Battery + |
| **BAT -** | Battery - |

---

### 2.9 70*55 Solar Panel

| Terminal | Connection Destination |
| --- | --- |
| **+** | MCP73833 IN + |
| **-** | MCP73833 IN - |

---

### 2.10 LCD Screen

* **Libraries:** `HardwareSerial.h`
* **Pinout Table:**

| LCD Pin | Connection Destination |
| --- | --- |
| **TX** | GPIO 18 |
| **RX** | GPIO 19 |
| **GND** | **D-MOSFET 1** (Ground Switching) |
| **5V** | Common 5V |

---

### 2.11 5V Step-Up Converter

| Port | Connection Destination |
| --- | --- |
| **IN +** | MCP73833 LOAD + |
| **IN -** | MCP73833 LOAD - |
| **OUT +** | Common 5V |
| **OUT -** | Common GND |

---

### 2.12 Miscellaneous

* Small Breadboard
* Du-Pont Wires (Jumper Cables)

---

### MOSFET Connection
<img width="1647" height="980" alt="MOSFET" src="https://github.com/user-attachments/assets/e171a122-be68-45d4-8205-140af52b6540" />

### **Table: MOSFET Wiring List**

| Component & Pin | Connection Destination | Notes & Function |
| --- | --- | --- |
| **MOSFET G (Pin 1)** | One end of Resistor **R1** ($220\Omega$) | **R1** is a series current-limiting resistor used to protect the ESP32 GPIO. |
| The other end of **R1** | **ESP32 GPIO Pin** (e.g., D4) | Control signal output from ESP32. |
| **MOSFET G (Pin 1)** | One end of Resistor **R2** ($10k\Omega$) | **R2** acts as a pull-down resistor. |
| The other end of **R2** | **MOSFET S (Pin 3)**, and connect to **GND** | Ensures the MOSFET remains OFF during ESP32 boot-up; bridges between G and S pins. |
|  |  |  |
| **MOSFET D (Pin 2)** | Device's **"R" Terminal** (Negative) | Controls the ground return path of the load device. |
| Device's **"I" Terminal** | **External Power Supply Positive (+)** | Connects the device to the positive voltage source. |
|  |  |  |
| **MOSFET S (Pin 3)** | **External Power Supply Negative (-)** | The common convergence point for all ground connections. |
| **ESP32 GND** | **External Power Supply Negative (-)** | **"Common Ground"** connection. Essential for operation. |

