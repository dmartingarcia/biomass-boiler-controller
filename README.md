# Lumber Boiler Manager

Management system for biomass boiler using ESP32 and PlatformIO.

## Features

- **Temperature Control**: PID control to regulate air intake and optimize combustion.
- **PID Auto-tuning**: Intelligent auto-tuning system to optimize the PID controller parameters.
- **Temperature Monitoring**: Four temperature sensors (boiler water, heating, combustion, and ambient).
- **Device Control**: Automatic control of pumps and fans based on temperatures.
- **Safety System (Killswitch)**: Automatic protection measure that activates all pumps and closes the air intake if the water temperature exceeds 90°C.
- **Connectivity Fault Tolerance**: The system continues to operate normally even without WiFi or MQTT connection.
- **Web Interface**: Web interface for remote monitoring and control.
- **GLCD Screen**: Local display of information on 128x64 GLCD screen.
- **Home Assistant Integration**: Complete integration with Home Assistant via MQTT.
- **OTA Updates**: Over-the-air firmware updates using ArduinoOTA.
- **Event Logging**: Storage and display of logs.

## Required Hardware

- ESP32 (LoLin S2 Mini o similar)
- 4 termistores NTC 10k con resistencias pull-up de 10k
- 4 relés de estado sólido
- 1 servo para control de entrada de aire
- Pantalla GLCD MKS MINI 12864 (SPI) - [Ver diagrama de conexiones](wiring_diagram.md)
- Fuente de alimentación adecuada

## Connections

### Temperature Sensors (NTC 10k Thermistors)

| Sensor | Analog Pin |
|--------|------------|
| Boiler Water | 34 |
| Heating | 35 |
| Combustion | 36 |
| Ambient | 39 |

Each thermistor must be connected with a 10k ohm pull-up resistor:

```text
3.3V
 |
[10k Resistor]
 |
 +--------> ADC Pin (GPIO34/35/36/39)
 |
[NTC 10k Thermistor]
 |
GND
```

### Relays

| Device | Pin |
|--------|-----|
| Boiler Pump | 15 |
| Heating Pump | 16 |
| Fans | 17 |
| Other | 18 |

### Servo

| Device | Pin |
|--------|-----|
| Air Intake | 19 |

### GLCD

| Conexión | Pin ESP32 | Pin MKS MINI 12864 |
|----------|---------|------------------|
| MOSI | GPIO40 | EXP1 Pin 3 (DOGLCD_MOSI) |
| SCK | GPIO38 | EXP1 Pin 5 (DOGLCD_SCK) |
| CS | GPIO36 | EXP1 Pin 4 (DOGLCD_CS) |
| RST | GPIO34 | EXP1 Pin 7 (DOGLCD_RESET) |
| DC/A0 | GPIO21 | EXP1 Pin 9 (DOGLCD_A0) |

## Complete Connection Diagram

```text
+------------------------------+
| ESP32 LoLin S2 Mini          |
+------------------------------+
| GPIO34 --> Boiler Water NTC (with 10k resistor to 3.3V)
| GPIO35 --> Heating NTC (with 10k resistor to 3.3V)
| GPIO36 --> Combustion NTC (with 10k resistor to 3.3V)
| GPIO39 --> Ambient NTC (with 10k resistor to 3.3V)
|
| GPIO15 --> Boiler Pump Relay
| GPIO16 --> Heating Pump Relay
| GPIO17 --> Fans Relay
| GPIO18 --> Additional Relay
|
| GPIO19 --> Air Intake Servo (signal)
| 5V ------> Air Intake Servo (power)
| GND -----> Air Intake Servo (ground)
|
| GPIO22 (SCL) --> GLCD SCL
| GPIO21 (SDA) --> GLCD SDA
| 5V -----------> GLCD VCC
| GND ----------> GLCD GND
|
| 5V -----------> Relay Power (VCC)
| GND ----------> Common ground
+------------------------------+
```

### Detailed Connections

#### 1. Temperature sensors (NTC 10k Thermistors)

Each NTC thermistor is connected in a voltage divider with a 10k ohm resistor:

```text
3.3V
 |
[10k Resistor]
 |
 +--------> ADC Pin (GPIO34/35/36/39)
 |
[NTC 10k Thermistor]
 |
GND
```

#### 2. Solid-state relays

```text
ESP32 GPIO (15/16/17/18) ---> [Relay Module] ---> Device (Pump/Fan)
                                |
                                +---> 5V (VCC)
                                +---> GND
```

#### 3. Air intake servo

```text
ESP32 GPIO19 ---> [Servo] Signal wire (typically yellow or orange)
ESP32 5V ------> [Servo] Power wire (typically red)
ESP32 GND -----> [Servo] Ground wire (typically brown or black)
```

### GLCD MKS MINI 12864 (SPI)

```text
ESP32 MOSI (GPIO40) ---> [MKS MINI 12864] EXP1 Pin 3 (DOGLCD_MOSI)
ESP32 SCK (GPIO38) ----> [MKS MINI 12864] EXP1 Pin 5 (DOGLCD_SCK)
ESP32 CS (GPIO36) ------> [MKS MINI 12864] EXP1 Pin 4 (DOGLCD_CS)
ESP32 RST (GPIO34) ----> [MKS MINI 12864] EXP1 Pin 7 (DOGLCD_RESET)
ESP32 DC (GPIO21) -----> [MKS MINI 12864] EXP1 Pin 9 (DOGLCD_A0)
ESP32 5V --------------> [MKS MINI 12864] EXP1 Pin 10 (VCC)
ESP32 GND -------------> [MKS MINI 12864] EXP1 Pin 8 (GND)
```

Ver el [diagrama detallado de conexiones](wiring_diagram.md) para información completa.

### Important assembly notes

1. **Power supply**: The ESP32 and components should receive a stable 5V power supply. Consider using a good quality power supply capable of supplying enough current for all components, especially when relays and servo are active.

2. **Protection**:
   - Use protection diodes on relays to prevent reverse currents
   - Consider using optocouplers to isolate relay control signals
   - If relays control inductive loads like motors, use flyback diodes

3. **NTC Thermistors**:
   - Use shielded cables for thermistors if distances are long
   - Consider adding filter capacitors (100nF) near analog inputs

4. **Safety**:
   - Ensure all high-voltage components are properly insulated
   - Use appropriate electrical boxes to contain electronic components
   - Consider adding fuses in the main power supply and in each load circuit

5. **Calibration**:
   - NTC thermistors will require calibration for accurate temperature readings
   - PID auto-tuning may require multiple attempts to find optimal parameters

## Configuration

The `include/config.h` file contains all configurable system settings:

- Pin definitions for all devices
- Temperature thresholds for pump activation
- PID parameters for air intake control
- Parameters for PID auto-tuning
- Web server and MQTT configuration
- WiFi reconnection intervals

WiFi credentials and OTA password should be configured in the `include/secrets.h` file.

## Operation

1. **System Startup**: Upon initialization, the system attempts to connect to the WiFi network. If it fails, it continues to operate in standalone mode without connectivity.

2. **Combustion Control**: The air intake is automatically regulated by PID to maintain combustion temperature at the target value.

3. **Device Activation**:
   - The boiler pump and fans are activated when combustion is detected.
   - The heating pump is activated when the water reaches a minimum temperature (50°C by default).

4. **Safety System**: The system includes an automatic killswitch that:
   - Activates when the boiler water temperature exceeds 90°C.
   - Activates all pumps to evacuate heat.
   - Completely closes the air intake to reduce combustion.
   - Sends alerts through the web interface and logs.
   - Automatically deactivates when the temperature returns to safe levels.

5. **Connectivity Fault Tolerance**:
   - If there is no WiFi connection, the system operates in standalone mode using only local control.
   - If there is WiFi but cannot connect to MQTT, the web interface works but there is no Home Assistant integration.
   - The system periodically attempts to reconnect to WiFi and MQTT without interrupting normal operation.
   - Connection retries have limits to avoid affecting system performance.

6. **Remote Control**: Target temperature and device status can be controlled from the web interface or from Home Assistant (if connected).

7. **Monitoring**:
   - The web interface shows all temperatures and statuses in real-time (if WiFi is available).
   - The local GLCD screen shows the same information regardless of connectivity.
   - Data is periodically sent to Home Assistant (if MQTT is available).

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/lumber-boiler-manager.git
   cd lumber-boiler-manager
   ```

2. Configure the `include/secrets.h` file with your WiFi credentials and OTA password:

   ```cpp
   #define WIFI_SSID "your_ssid"
   #define WIFI_PASSWORD "your_password"
   #define OTA_PASSWORD "ota_password"
   ```

3. Compile and upload the firmware:

   ```bash
   pio run --target upload
   ```

4. Upload the file system (for the web interface):

   ```bash
   pio run --target uploadfs
   ```

## Usage

1. Once installed, the device will connect to the configured WiFi network.
2. You can access the web interface at `http://lumber-boiler.local` or using the assigned IP address.
3. Integration with Home Assistant should be automatic if you are using MQTT Discovery.

## PID Auto-tuning System

The system includes an advanced auto-tuning functionality for the PID controller that regulates the boiler's air intake, automatically optimizing combustion.

### How PID auto-tuning works

The implemented auto-tuning uses the **Ziegler-Nichols relay method** which works as follows:

1. During the auto-tuning process, the system generates small controlled oscillations in the combustion temperature.

2. The algorithm analyzes these oscillations (amplitude and period) to determine the dynamic characteristics of the boiler.

3. Based on these characteristics, it automatically calculates the optimal values for Kp, Ki, and Kd.

4. Upon completion, these values are applied to the PID controller, providing more precise and stable temperature control.

### When to use auto-tuning

1. **Initial setup**: When installing the system for the first time.

2. **Changes in conditions**: If you change the biomass type or make modifications to the boiler.

3. **Suboptimal control**: If you observe that the combustion temperature oscillates too much or is not stable.

### How to use auto-tuning

1. **Access the web interface** of the system through `http://lumber-boiler.local` or the device's IP address.

2. **Wait for stable conditions**: Start auto-tuning when the boiler is operating under normal conditions, preferably with a stable load.

3. **Start the process**: In the "PID Configuration" section of the web interface, click the "Start Auto-tuning" button.

4. **Wait for completion**: The process can take 10-20 minutes. During this time, you'll see the air intake oscillating and the combustion temperature fluctuating.

5. **Monitoring**: The web interface will show the current auto-tuning status and, once completed, the newly calculated PID parameters.

### Recommendations

1. **Supervision**: During auto-tuning, maintain supervision over the boiler, as temperature oscillations could affect fire behavior.

2. **Cancellation**: If the behavior during auto-tuning is not safe or desirable, you can cancel it at any time using the button in the web interface.

3. **Frequency**: It is not necessary to perform auto-tuning frequently, only when conditions change significantly.

### Configuration parameters

The following auto-tuning parameters can be configured in the `include/config.h` file:

```cpp
// PID auto-tuning configuration
#define PID_CONTROL_TYPE               1     // 0=PID, 1=PI, 2=P (PI recommended for temperature control)
#define PID_NOISE_BAND                 1.0   // Noise band for auto-tuning (degrees C)
#define PID_OUTPUT_STEP                50    // Output change during auto-tuning (0-180)
#define PID_LOOKBACK_SEC               30    // Seconds of history to analyze during auto-tuning
```

## Contributing

Contributions are welcome. Please feel free to submit pull requests or open issues to improve the project.

## License

This project is licensed under the MIT License.
