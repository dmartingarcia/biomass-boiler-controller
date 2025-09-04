#ifndef FS_HELPER_H
#define FS_HELPER_H

#include <Arduino.h>
#include <LittleFS.h>

// Clase helper para gestionar el sistema de archivos
class FSHelper {
public:
    static bool initializeLittleFS() {
        // Intentar montar el sistema de archivos
        if (!LittleFS.begin(false)) {
            Serial.println("Error al montar LittleFS. Intentando formatear...");

            // Si falla, intentar formatear
            if (!LittleFS.format()) {
                Serial.println("Error al formatear LittleFS. No se puede inicializar el sistema de archivos.");
                return false;
            }

            // Intentar montar de nuevo tras formatear
            if (!LittleFS.begin()) {
                Serial.println("Error al montar LittleFS después de formatear. Revise la configuración del hardware.");
                return false;
            }

            Serial.println("LittleFS formateado y montado correctamente.");
        } else {
            Serial.println("LittleFS montado correctamente.");
        }

        // Mostrar información sobre el sistema de archivos
        printFSInfo();

        return true;
    }

    static void printFSInfo() {
        #ifdef ESP32
        // En ESP32, obtener información básica
        Serial.print("Total space: ");
        Serial.print(LittleFS.totalBytes() / 1024);
        Serial.println(" KB");
        Serial.print("Used space: ");
        Serial.print(LittleFS.usedBytes() / 1024);
        Serial.println(" KB");
        #endif
    }
};

#endif // FS_HELPER_H
