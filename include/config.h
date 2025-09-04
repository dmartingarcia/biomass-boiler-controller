#ifndef CONFIG_H
#define CONFIG_H

// Pin definitions for 10k NTC thermistors
#define NTC_BOILER_WATER_PIN           33
#define NTC_HEATING_PIN                35
#define NTC_BURNING_PIN                37
#define NTC_AMBIENT_PIN                39

// Series resistor for NTC thermistors (in ohms)
#define NTC_SERIES_RESISTOR            10000

// Pin definitions for solid state relays
#define RELAY_BOILER_PUMP              12
#define RELAY_HEATING_PUMP             11
#define RELAY_FANS                     9
#define RELAY_OTHER                    7

// Pin definition for air intake servo
#define SERVO_AIR_INTAKE               19

// Default air intake servo range (in degrees)
#define DEFAULT_SERVO_MIN              0     // Minimum servo position (0% air)
#define DEFAULT_SERVO_MAX              180   // Maximum servo position (100% air)

// Pin definitions for MKS MINI 12864 GLCD display (SPI Interface)
// Conexiones entre ESP32 y MKS MINI 12864:
//
// --- Conector EXP1 del MKS MINI 12864 ---
// MOSI (ESP32) -> EXP1 Pin 3 (DOGLCD_MOSI)
// SCK (ESP32)  -> EXP1 Pin 5 (DOGLCD_SCK)
// CS (ESP32)   -> EXP1 Pin 4 (DOGLCD_CS)
// RST (ESP32)  -> EXP1 Pin 7 (DOGLCD_RESET)
// DC/A0 (ESP32)-> EXP1 Pin 9 (DOGLCD_A0)
// 5V           -> EXP1 Pin 10 (VCC)
// GND          -> EXP1 Pin 8 (GND)
//
// Nota: La pantalla MKS MINI 12864 utiliza el controlador ST7567 y soporta interfaz SPI de 4 hilos.
// Para mejor compatibilidad, usar el constructor U8G2_ST7567_OS12864_F_4W_SW_SPI para SPI por software
// cuando no usamos los pines SPI hardware estándar.
#define GLCD_MOSI                      40    // SPI MOSI pin
#define GLCD_SCK                       38    // SPI SCK pin
#define GLCD_CS                        36    // SPI CS pin
#define GLCD_RST                       34    // Reset pin (llamado GLCD_RESET también)
#define GLCD_DC                        21    // Data/Command pin (llamado GLCD_A0 también)

// Temperature threshold configuration (in Celsius degrees)
#define BURNING_TEMP_THRESHOLD         100.0  // Minimum temperature to consider combustion is occurring
#define BOILER_WATER_TEMP_THRESHOLD    40.0  // Minimum temperature to activate the heating pump
#define BOILER_WATER_CRITICAL_TEMP     90.0  // Critical temperature to activate the safety killswitch

// PID configuration for air intake servo control
#define PID_KP                         2.0
#define PID_KI                         0.1
#define PID_KD                         1.0
#define PID_SAMPLE_TIME                1000  // Sampling time in milliseconds

// Configuration for PID auto-tuning
#define PID_CONTROL_TYPE               1     // 0=PID, 1=PI, 2=P (PI recommended for temperature control)
#define PID_NOISE_BAND                 1.0   // Noise band for auto-tuning (degrees C)
#define PID_OUTPUT_STEP                50    // Output change during auto-tuning (0-180)
#define PID_LOOKBACK_SEC               30    // Seconds of history to analyze during auto-tuning

// Default setting for desired burning temperature
#define DEFAULT_TARGET_BURNING_TEMP    85.0  // Default optimal combustion temperature

// Configuration for log buffer
#define LOG_BUFFER_SIZE                100  // Number of entries in the circular buffer

// Web interface configuration
#define HOSTNAME                       "lumber-boiler"
#define WEB_SERVER_PORT                80
#define WIFI_RECONNECT_INTERVAL        60000  // Interval to attempt WiFi reconnection (ms)

// MQTT configuration for Home Assistant
#define MQTT_SERVER                    "homeassistant.local"
#define MQTT_PORT                      1883
#define MQTT_USER                      "homeassistant"
#define MQTT_PASSWORD                  "homeassistant"
#define MQTT_BASE_TOPIC                "lumber-boiler"
#define MQTT_CLIENT_ID                 "lumber-boiler-manager"
#define MQTT_PUBLISH_INTERVAL          10000  // Publishing interval in milliseconds

#endif // CONFIG_H
