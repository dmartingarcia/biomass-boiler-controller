# Diagrama de Conexiones — Biomass Boiler Controller

Plataforma: **Lolin S2 Mini** (ESP32-S2)

## Sensores de temperatura

### Dallas DS18B20 — bus 1-Wire (múltiples sensores)

| DS18B20 Pin | Conexión            |
|-------------|---------------------|
| VDD         | 3.3V                |
| GND         | GND                 |
| DQ (data)   | GPIO 11             |

> Resistencia pull-up de **4.7 kΩ** entre DQ y 3.3V obligatoria.  
> Todos los sensores comparten el mismo pin de datos (bus 1-Wire).  
> Los sensores se identifican por su dirección única de 64 bits.  
> Asignar cada dirección a un rol (agua caldera, calefacción, ambiente) desde la web.

### MAX6675 — Termopar tipo K (temperatura de combustión)

| MAX6675 Pin | GPIO S2 Mini |
|-------------|--------------|
| VCC         | 3.3V         |
| GND         | GND          |
| SCK         | GPIO 7       |
| SO (MISO)   | GPIO 9       |
| CS          | GPIO 12      |

> El MAX6675 mide la temperatura de combustión (hasta ~1024°C).  
> Usa SPI bit-bang (software). Rango útil: 0–1024°C, resolución 0.25°C.

## Pantalla OLED (SSD1306 / SH1106, 128×64, I2C)

| OLED Pin | GPIO S2 Mini |
|----------|--------------|
| VCC      | 3.3V         |
| GND      | GND          |
| SDA      | GPIO 33      |
| SCL      | GPIO 35      |

> El tipo de controlador se elige en `config.h`: con `#define OLED_TYPE_SH1106` (activo por defecto)
> se usa SH1106; coméntalo para SSD1306.

## Relés (salidas)

| Función           | GPIO S2 Mini | Notas                                  |
|-------------------|--------------|----------------------------------------|
| Relay EN / Other  | GPIO 0       | Boot strapping pin — activo HIGH recomendado |
| Bomba caldera     | GPIO 1       |                                        |
| Bomba calefacción | GPIO 2       |                                        |
| Ventiladores      | GPIO 3       |                                        |

> GPIO 0 es el pin de strapping de boot. Si tu módulo de relés es active-LOW,
> asegúrate de que GPIO 0 no quede en LOW durante el arranque o el ESP entrará
> en modo descarga. Se recomienda módulo de relés con optoacoplador active-HIGH
> o añadir resistencia pull-up externa.

## Servo — Entrada de aire

| Conexión | GPIO S2 Mini |
|----------|--------------|
| Signal   | GPIO 19      |
| VCC      | 5V (externo) |
| GND      | GND          |

## Alimentación general

- ESP32 S2 Mini: USB o 5V en pin VIN
- Sensores Dallas y OLED: 3.3V del regulador de la placa
- Servo y relés: alimentación externa 5V/12V según módulo

## Archivos de configuración en LittleFS

| Archivo                | Contenido                                      |
|------------------------|------------------------------------------------|
| `/sensor_config.json`  | Asignación de sensores Dallas a roles          |

Ejemplo de `/sensor_config.json`:
```json
{
  "boiler_water": "28FF7D5A41180264",
  "heating":      "28FF1234ABCD5678",
  "ambient":      "28FFABCDEF123456"
}
```

## API REST

| Método | Endpoint             | Descripción                                 |
|--------|----------------------|---------------------------------------------|
| GET    | /api/status          | Estado general del sistema                  |
| GET    | /api/logs            | Buffer de logs                              |
| POST   | /api/settings        | Configurar target temp, PID, autotune       |
| GET    | /api/dallas/sensors  | Lista todos los sensores Dallas + temps     |
| POST   | /api/dallas/assign   | `{"role":"boiler_water","address":"28FF…"}` |
| POST   | /api/dallas/scan     | Re-escanea el bus 1-Wire                    |
