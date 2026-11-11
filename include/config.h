#ifndef CONFIG_H
#define CONFIG_H

// 1-Wire bus for Dallas DS18B20 temperature sensors
#define ONE_WIRE_PIN                   11

// MAX6675 thermocouple (software SPI)
#define MAX6675_SCK                    7
#define MAX6675_SO                     9     // MISO / data out
#define MAX6675_CS                     12

// Solid state relay pins
// EN = GPIO 0 (labeled EN/boot button on Lolin S2 Mini – verify this matches your wiring)
#define RELAY_OTHER                    0     // EN relay
#define RELAY_BOILER_PUMP              1
#define RELAY_HEATING_PUMP             2
#define RELAY_FANS                     3

// Air intake servo
#define SERVO_AIR_INTAKE               19

// Default air intake servo range (degrees)
#define DEFAULT_SERVO_MIN              0
#define DEFAULT_SERVO_MAX              180

// OLED display (I2C, typically SSD1306 128x64)
#define OLED_SDA                       33
#define OLED_SCL                       35

// OLED controller type — define OLED_TYPE_SH1106 if your display uses SH1106
// (many 0.96" OLEDs sold as "SSD1306" are actually SH1106 — try this first if nothing shows)
#define OLED_TYPE_SH1106

// Target burning temperature range (for UI slider)
#define TARGET_TEMP_MIN                30.0
#define TARGET_TEMP_MAX                300.0

// Temperature thresholds (Celsius)
#define BURNING_TEMP_THRESHOLD         100.0
// Combustion is "on" above BURNING_TEMP_THRESHOLD and only turns "off" again once the
// flue gas falls below (threshold - BURNING_HYSTERESIS), to avoid on/off chatter
#define BURNING_HYSTERESIS             5.0
#define BOILER_WATER_TEMP_THRESHOLD    40.0
#define BOILER_WATER_CRITICAL_TEMP     90.0

// PID configuration
#define PID_KP                         2.0
#define PID_KI                         0.1
#define PID_KD                         1.0
#define PID_SAMPLE_TIME                1000

// PID auto-tuning
#define PID_CONTROL_TYPE               1
#define PID_NOISE_BAND                 1.0
#define PID_OUTPUT_STEP                50
#define PID_LOOKBACK_SEC               30

#define DEFAULT_TARGET_BURNING_TEMP    85.0

// Temperature the air-intake PID regulates (index in history.h channel order):
// 0 boiler_supply, 1 boiler_return, 2 boiler_iron, 3 floor_supply, 4 floor_return,
// 5 mix_supply, 6 ambient_low, 7 ambient_high, 8 burning (flue-gas thermocouple)
#define DEFAULT_PID_SOURCE             8

#define LOG_BUFFER_SIZE                100

// Network
#define HOSTNAME                       "lumber-boiler"
#define WEB_SERVER_PORT                80
#define WIFI_RECONNECT_INTERVAL        60000

// WiFi AP mode fallback (when no config or connection fails)
#define AP_SSID                        "LumberBoiler-Setup"
#define AP_PASSWORD                    ""
#define WIFI_CONFIG_FILE               "/wifi_config.json"

// Default static IP (used as starting value in the WiFi config UI)
#define DEFAULT_STATIC_IP              "192.168.0.188"
#define DEFAULT_STATIC_GATEWAY         "192.168.0.1"
#define DEFAULT_STATIC_SUBNET          "255.255.255.0"
#define DEFAULT_STATIC_DNS             "192.168.0.1"

// Temperature history (RAM ring buffers, lost on reboot)
// Fine tier: one sample per minute for the last hour.
// Coarse tier: one sample per 5 minutes (average) for the last 24 hours.
#define HISTORY_FINE_COUNT             60
#define HISTORY_COARSE_COUNT           288
#define HISTORY_FINE_INTERVAL_MS       60000UL
#define HISTORY_COARSE_EVERY           5       // fine samples per coarse sample

// Sensor validity — Dallas readings at or below this value are considered errors
#define DALLAS_MIN_VALID_TEMP          -20.0
#define DALLAS_MAX_VALID_TEMP          120.0

// Boiler-supply sensor fault: this many consecutive seconds without a valid reading
// (while the sensor is assigned) puts the system in safe mode
#define WATER_SENSOR_FAULT_SECS        10

// Pump / relay debounce — minimum state hold time (minutes)
#define RELAY_DEBOUNCE_MIN             5

// Keep boiler pump running while boiler iron body is above this temperature (°C)
// — extracts residual heat from the cast-iron mass after combustion ends
#define BOILER_IRON_PUMP_THRESHOLD     50.0

// Heating pump activates only when (ida_caldera − retorno_suelo) exceeds this (°C)
// — ensures there is useful heat to transfer to the floor circuit
#define FLOOR_DELTA_THRESHOLD          5.0

// MQTT
#define MQTT_SERVER                    "homeassistant.local"
#define MQTT_PORT                      1883
#define MQTT_USER                      "homeassistant"
#define MQTT_PASSWORD                  "homeassistant"
#define MQTT_BASE_TOPIC                "lumber-boiler"
#define MQTT_CLIENT_ID                 "lumber-boiler-manager"
#define MQTT_PUBLISH_INTERVAL          10000

#endif // CONFIG_H
