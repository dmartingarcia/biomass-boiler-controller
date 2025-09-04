# Diagrama de Conexiones para MKS MINI 12864 GLCD

Este documento proporciona información detallada sobre cómo conectar la pantalla MKS MINI 12864 GLCD a un ESP32 para el proyecto Lumber Boiler Manager.

## Especificaciones Técnicas

- **Controlador**: ST7567
- **Resolución**: 128x64 píxeles
- **Interfaz**: SPI de 4 hilos (usando SPI por software)
- **Retroiluminación**: LED (controlable)
- **Alimentación**: 5V

## Diagrama de Pines

### Conector EXP1 (Interfaz SPI)

| Pin EXP1 | Nombre en MKS MINI | Conectar a ESP32  | Configuración |
|----------|-------------------|-------------------|---------------|
| 1        | BEEPER            | No conectado      | -             |
| 2        | BTN_ENC           | No conectado      | -             |
| 3        | DOGLCD_MOSI       | GLCD_MOSI (40)    | SPI MOSI      |
| 4        | DOGLCD_CS         | GLCD_CS (36)      | SPI CS        |
| 5        | DOGLCD_SCK        | GLCD_SCK (38)     | SPI SCK       |
| 6        | LCD_PINS_EN       | No conectado      | -             |
| 7        | DOGLCD_RESET      | GLCD_RST (34)     | Reset         |
| 8        | GND               | GND               | Tierra        |
| 9        | DOGLCD_A0         | GLCD_DC (21)      | Data/Command  |
| 10       | VCC               | 5V                | Alimentación  |

## Notas Importantes

1. **Librería U8g2**: La pantalla utiliza el controlador ST7567, y se usa el constructor `U8G2_ST7567_OS12864_F_4W_SW_SPI` para SPI por software.

2. **Pines SPI Personalizados**: Se están utilizando pines personalizados para SPI (40, 38, 36, 34, 21) en lugar de los pines SPI hardware estándar del ESP32.

3. **Rotación de Pantalla**: Si la orientación no es correcta, se puede ajustar cambiando el parámetro de rotación en el constructor (`U8G2_R0`, `U8G2_R1`, `U8G2_R2`, o `U8G2_R3`).

4. **Contraste**: Puede ser necesario ajustar el contraste con `u8g2.setContrast(160)` en el método `begin()`.

5. **Voltaje**: Aunque la lógica del ESP32 es de 3.3V, la pantalla MKS MINI 12864 es compatible con estos niveles lógicos. La alimentación debe ser de 5V para el funcionamiento correcto de la retroiluminación.

## Ejemplo de Código

```cpp
// Inicialización en constructor (SPI por software)
Display() : u8g2(U8G2_R0, GLCD_SCK, GLCD_MOSI, GLCD_CS, GLCD_DC, GLCD_RST) {
    // ...inicialización...
}

// Configuración de pantalla
void begin() {
    // Inicializar la pantalla
    u8g2.begin();
    u8g2.setContrast(160);  // Ajustar contraste según sea necesario
    // ...más configuración...
}
```

## Referencias

- [Documentación U8g2](https://github.com/olikraus/u8g2/wiki)
- [Datasheet ST7567](https://www.newhavendisplay.com/app_notes/ST7567.pdf)
- [MKS MINI 12864 Pinout](https://github.com/makerbase-mks/MKS-MINI12864)
