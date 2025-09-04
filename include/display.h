#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <WiFi.h>
#include "config.h"

class Display {
public:
    // Enumeration for different display screens
    enum ScreenType {
        SCREEN_MAIN,         // Main data screen
        SCREEN_NETWORK_INFO  // Network information screen
    };

    // Inicialización para MKS MINI 12864 con interfaz SPI
    // El constructor es para ST7567 controller usado en la pantalla MKS MINI 12864
    // Uso de SPI por software para poder usar pines personalizados
    Display() : u8g2(U8G2_R0, GLCD_SCK, GLCD_MOSI, GLCD_CS, GLCD_DC, GLCD_RST) {
        currentScreen = SCREEN_MAIN;
        lastScreenToggle = 0;
        screenToggleInterval = 5000; // Default a 5 segundos
    }

    void begin() {
        // Inicializar la pantalla
        u8g2.begin();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setDrawColor(1);
        u8g2.setFontPosTop();
        u8g2.setContrast(160); // Ajustar contraste para la pantalla ST7567

        // Mostrar pantalla de bienvenida
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_8x13B_tf);
        u8g2.drawStr(5, 5, "Lumber Boiler");
        u8g2.drawStr(15, 25, "Manager");
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(25, 45, "Starting...");
        u8g2.sendBuffer();
        delay(1000);  // Mostrar la pantalla de bienvenida por 1 segundo
    }

    // Update method - to be called regularly from the main loop
    void update(float boilerWaterTemp, float heatingTemp, float burningTemp, float ambientTemp,
                bool boilerPump, bool heatingPump, bool fans, float targetBurningTemp, int airIntakePosition) {

        // Check if it's time to toggle screens
        unsigned long currentMillis = millis();
        if (currentMillis - lastScreenToggle >= screenToggleInterval) {
            lastScreenToggle = currentMillis;
            // Toggle between screens
            currentScreen = (currentScreen == SCREEN_MAIN) ? SCREEN_NETWORK_INFO : SCREEN_MAIN;
        }

        // Display the current screen
        u8g2.clearBuffer();
        if (currentScreen == SCREEN_MAIN) {
            showMainScreen(boilerWaterTemp, heatingTemp, burningTemp, ambientTemp,
                          boilerPump, heatingPump, fans, targetBurningTemp, airIntakePosition);
        } else {
            showNetworkInfo();
        }
        u8g2.sendBuffer();
    }

    // Force a specific screen to be shown
    void setScreen(ScreenType screen) {
        currentScreen = screen;
        lastScreenToggle = millis(); // Reset the toggle timer
    }

    // Configure the screen toggle interval
    void setScreenToggleInterval(unsigned long interval) {
        screenToggleInterval = interval;
    }

    bool isAvailable() {
        return true;  // Always return true as this is a simple implementation
    }

    void showAlert(const String& title, const String& line1, const String& line2, const String& line3) {
        u8g2.clearBuffer();

        // Title in bold
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.drawStr(0, 0, title.c_str());
        u8g2.drawLine(0, 13, 128, 13);

        // Alert text
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 16, line1.c_str());
        u8g2.drawStr(0, 28, line2.c_str());
        u8g2.drawStr(0, 40, line3.c_str());

        u8g2.sendBuffer();
    }

    // Method to show network information screen
    void showNetworkInfo() {
        u8g2.clearBuffer();

        // Title
        u8g2.setFont(u8g2_font_7x13_tf);
        u8g2.drawStr(0, 0, "Network Info");
        u8g2.drawLine(0, 13, 128, 13);

        // Return to normal font
        u8g2.setFont(u8g2_font_6x10_tf);

        // WiFi Status
        if (WiFi.status() == WL_CONNECTED) {
            // Connected to WiFi
            u8g2.drawStr(0, 16, "WiFi: Connected");

            // Show SSID
            String ssid = WiFi.SSID();
            if (ssid.length() > 16) {
                ssid = ssid.substring(0, 14) + "..";
            }
            u8g2.drawStr(0, 26, ("SSID: " + ssid).c_str());

            // Show IP
            String ip = WiFi.localIP().toString();
            u8g2.drawStr(0, 36, ("IP: " + ip).c_str());

            // Show Signal strength
            int rssi = WiFi.RSSI();
            String signal = "Signal: ";
            if (rssi > -50) signal += "Excellent";
            else if (rssi > -60) signal += "Good";
            else if (rssi > -70) signal += "Fair";
            else signal += "Weak";
            u8g2.drawStr(0, 46, signal.c_str());

            // Show Host name
            u8g2.drawStr(0, 56, ("Host: " + String(HOSTNAME)).c_str());
        } else {
            // Not connected
            u8g2.drawStr(0, 26, "WiFi: Not Connected");
            u8g2.drawStr(0, 36, "Operating in");
            u8g2.drawStr(0, 46, "standalone mode");
        }

        u8g2.sendBuffer();
    }

private:
    // Method to show the main information screen
    void showMainScreen(float boilerWaterTemp, float heatingTemp, float burningTemp, float ambientTemp,
                      bool boilerPump, bool heatingPump, bool fans, float targetBurningTemp, int airIntakePosition) {
        u8g2.clearBuffer();

        // Title
        u8g2.setFont(u8g2_font_7x13_tf);
        u8g2.drawStr(0, 0, "Biomass Boiler");
        u8g2.drawLine(0, 13, 128, 13);

        // Return to normal font
        u8g2.setFont(u8g2_font_6x10_tf);

        // Temperatures
        char buffer[32];
        sprintf(buffer, "Water: %.1fC", boilerWaterTemp);
        u8g2.drawStr(0, 16, buffer);

        sprintf(buffer, "Heat: %.1fC", heatingTemp);
        u8g2.drawStr(0, 26, buffer);

        sprintf(buffer, "Burn: %.1fC", burningTemp);
        u8g2.drawStr(0, 36, buffer);

        sprintf(buffer, "Amb: %.1fC", ambientTemp);
        u8g2.drawStr(0, 46, buffer);

        // Status
        u8g2.drawStr(70, 16, "Pump B:");
        u8g2.drawStr(115, 16, boilerPump ? "ON" : "OFF");

        u8g2.drawStr(70, 26, "Pump H:");
        u8g2.drawStr(115, 26, heatingPump ? "ON" : "OFF");

        u8g2.drawStr(70, 36, "Fans:");
        u8g2.drawStr(115, 36, fans ? "ON" : "OFF");

        // Show IP Address if connected to WiFi
        if (WiFi.status() == WL_CONNECTED) {
            String ipStr = WiFi.localIP().toString();
            u8g2.drawStr(70, 46, "IP:");
            u8g2.drawStr(85, 46, ipStr.c_str());
        }

        sprintf(buffer, "Tgt: %.1fC", targetBurningTemp);
        u8g2.drawStr(0, 56, buffer);

        sprintf(buffer, "Air: %d%%", airIntakePosition);
        u8g2.drawStr(70, 56, buffer);

        u8g2.sendBuffer();
    }

    U8G2_ST7567_OS12864_F_4W_SW_SPI u8g2;
    ScreenType currentScreen;
    unsigned long lastScreenToggle;
    unsigned long screenToggleInterval;
};

#endif // DISPLAY_H