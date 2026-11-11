# Controlador de caldera de biomasa (ESP32-S2)

Firmware para una **Lolin S2 Mini** que regula una caldera de biomasa: abre/cierra la entrada de aire con un servo mediante un PID, gobierna dos bombas y unos ventiladores, y ofrece una interfaz web (control manual, gráficas, configuración) e integración con Home Assistant por MQTT.

## Hardware

| Función | GPIO | Notas |
|---|---|---|
| Bus 1-Wire (8 × DS18B20) | 11 | pull-up de 4,7 kΩ a 3,3 V |
| Termopar MAX6675 (SCK / SO / CS) | 7 / 9 / 12 | SPI por software |
| OLED 128×64 I2C (SDA / SCL) | 33 / 35 | SH1106 por defecto (`OLED_TYPE_SH1106` en `config.h`) |
| Servo de la entrada de aire | 19 | |
| Relés (SSR, activos en alto) | 0, 1, 2, 3 | canales 1–4; qué función lleva cada uno se asigna desde la UI |

Cableado completo en [wiring_diagram.md](wiring_diagram.md).

## Lógica de control

- **Combustión**: se detecta con el termopar de gases. «Quemando» por encima de `burning_temp_threshold` (100 °C por defecto) y se apaga 5 °C por debajo (histéresis).
- **Bomba de caldera**: ON si hay combustión, el hierro de la caldera está caliente (> 50 °C) o el agua está caliente (> 40 °C).
- **Bomba de calefacción**: ON si la bomba de caldera funciona, el agua está caliente y (ida caldera − retorno suelo) supera el delta mínimo.
- **Ventiladores**: ON mientras hay combustión.
- **Entrada de aire (PID)**: regula la temperatura que elijas en *Config → Objetivo del PID* hacia un objetivo en °C. Por defecto es la **ida de caldera (agua)**; los gases del termopar solo sirven para *detectar* la combustión, que es lo que activa el PID. Sin combustión el aire se cierra. El PID es propio ([pid_controller.h](include/pid_controller.h)): anti-windup, derivada sobre la medida y arranque sin saltos.
- **Debounce de relés** (5 min por defecto) en automático, para no ciclar bombas.

### Seguridad
- **Temperatura crítica** (ida caldera > 90 °C): bombas y ventiladores forzados, aire cerrado, y cancela el modo manual.
- **Sensor de ida caldera sin lectura** (10 s seguidos, si hay un sensor asignado): modo seguro. Con combustión o hierro caliente fuerza las bombas y cierra el aire; avisa en la UI y en el log.
- Un sensor que falla se muestra como `ERR` (no se repite el último valor).
- **Modo manual**: sin timeout; dura hasta que pulses «Automático» (no sobrevive a un reinicio). La emergencia por temperatura crítica lo cancela y lo bloquea mientras dure.

## Interfaz web

`http://<ip>/` (por defecto `192.168.0.188`, `lumber-boiler.local`):

- **Panel**: temperaturas, actuadores, estado del PID y última hora.
- **Manual**: relés con su estado real, servo en vivo y **autotest** (pulsa cada relé 1 s, prueba pantalla, termopar y Dallas).
- **Gráficas**: histórico de temperaturas, 1 muestra/min la última hora y 1 cada 5 min hasta 24 h (en RAM: se pierde al reiniciar).
- **Config**: PID y su fuente, umbrales, servo, debounce, MQTT y **asignación de relés a canales**.
- **Diagnóstico**: sensores Dallas y roles (con direcciones), OLED, MQTT, backup (export/import), causa del último reinicio y logs.

## Build y subida

Requiere [PlatformIO](https://platformio.org/).

```sh
pio run                                  # compilar
pio run -t upload -t uploadfs            # USB: firmware + interfaz (data/)
pio run -e ota -t upload                 # OTA: firmware
pio run -e ota -t uploadfs               # OTA: interfaz (data/index.html)
```

El firmware y la interfaz van en imágenes distintas: sube las dos tras cambiar la API. La partición de datos es de 128 KB y `index.html` ocupa ~64 KB.

Credenciales por defecto en `include/secrets.h` (WiFi y contraseña OTA); el WiFi también se puede cambiar desde el modo AP (`LumberBoiler-Setup`, `/wifi-setup`).

### Tests
Lógica sin hardware (PID, histórico, mapa de relés, autoajuste):
```sh
test/host/run.sh
```

## API (resumen)

| Método y ruta | Qué hace |
|---|---|
| `GET /api/status` | estado completo (temperaturas, relés, modo, PID, alarmas, motivo de reinicio) |
| `GET /api/history` | histórico en formato columnar |
| `POST /api/manual` `{active}` | activa/desactiva el modo manual |
| `POST /api/manual/actuators` | relés y servo (solo en manual) |
| `POST /api/selftest` · `GET /api/selftest` | lanza / consulta el autotest (solo en manual) |
| `GET/POST /api/config` | ajustes (con validación; rechaza valores incoherentes) |
| `GET /api/config/export` · `POST /api/config/import` · `POST /api/config/reset` | backup (ajustes, mapa de relés, roles Dallas) |
| `GET/POST /api/relays/map` | asignación función → canal de relé |
| `GET /api/dallas/sensors` · `POST /api/dallas/assign` · `POST /api/dallas/scan` | sensores y roles |
| `GET /api/mqtt/status` · `POST /api/mqtt/retry` | estado de MQTT |

> **Orden de rutas**: ESPAsyncWebServer casa por prefijo y gana el primer handler registrado. Las subrutas (`/api/config/export`, `/api/manual/actuators`) deben registrarse **antes** que su ruta base, o ésta se las traga.

## Notas conocidas
- El bus 1-Wire y el termopar solo los toca el bucle principal; la web lee valores en caché.
- El GPIO 0 es un pin de arranque del chip; tras reiniciar por OTA el ESP ha llegado a quedarse sin responder hasta cortarle la alimentación.
- Sin autenticación en la API web (red local de confianza).
