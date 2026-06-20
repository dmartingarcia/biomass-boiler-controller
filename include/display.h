#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include "config.h"

#ifdef OLED_TYPE_SH1106
  #define _U8G2_TYPE U8G2_SH1106_128X64_NONAME_F_SW_I2C
#else
  #define _U8G2_TYPE U8G2_SSD1306_128X64_NONAME_F_SW_I2C
#endif

// OLED over software I2C on custom SDA/SCL pins.
// If nothing shows, try defining OLED_TYPE_SH1106 in config.h
class Display {
public:
    enum ScreenType {
        SCREEN_MAIN,
        SCREEN_NETWORK_INFO
    };

    Display() : u8g2(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE), available(false) {
        currentScreen = SCREEN_MAIN;
        lastScreenToggle = 0;
        screenToggleInterval = 5000;
    }

    void begin() {
        available = u8g2.begin();
        if (!available) return;

        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setDrawColor(1);
        u8g2.setFontPosTop();

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_8x13B_tf);
        u8g2.drawStr(5, 5, "Biomass Boiler");
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(25, 30, "Starting...");
        u8g2.sendBuffer();
        delay(1000);
    }

    // Self-test helper: show a full-screen message with up to 4 lines
    void showMessage(const char* title, const char* l1 = "", const char* l2 = "", const char* l3 = "") {
        if (!available) return;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.drawStr(0, 0, title);
        u8g2.drawLine(0, 13, 128, 13);
        u8g2.setFont(u8g2_font_6x10_tf);
        if (l1 && l1[0]) u8g2.drawStr(0, 16, l1);
        if (l2 && l2[0]) u8g2.drawStr(0, 28, l2);
        if (l3 && l3[0]) u8g2.drawStr(0, 40, l3);
        u8g2.sendBuffer();
    }

    void update(float boilerSupplyTemp, float floorSupplyTemp, float burningTemp, float ambientLowTemp,
                bool boilerPump, bool heatingPump, bool fans, float targetBurningTemp, int airIntakePosition) {
        if (!available) return;
        unsigned long now = millis();
        if (now - lastScreenToggle >= screenToggleInterval) {
            lastScreenToggle = now;
            currentScreen = (currentScreen == SCREEN_MAIN) ? SCREEN_NETWORK_INFO : SCREEN_MAIN;
        }

        u8g2.clearBuffer();
        if (currentScreen == SCREEN_MAIN)
            showMainScreen(boilerSupplyTemp, floorSupplyTemp, burningTemp, ambientLowTemp,
                           boilerPump, heatingPump, fans, targetBurningTemp, airIntakePosition);
        else
            showNetworkInfo();
        u8g2.sendBuffer();
    }

    void setScreen(ScreenType screen) {
        currentScreen = screen;
        lastScreenToggle = millis();
    }

    void setScreenToggleInterval(unsigned long interval) {
        screenToggleInterval = interval;
    }

    bool isAvailable() { return available; }

    void showAlert(const String& title, const String& l1, const String& l2, const String& l3) {
        if (!available) return;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x13B_tf);
        u8g2.drawStr(0, 0, title.c_str());
        u8g2.drawLine(0, 13, 128, 13);
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 16, l1.c_str());
        u8g2.drawStr(0, 28, l2.c_str());
        u8g2.drawStr(0, 40, l3.c_str());
        u8g2.sendBuffer();
    }

    void showNetworkInfo() {
        if (!available) return;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x13_tf);
        u8g2.drawStr(0, 0, "Network Info");
        u8g2.drawLine(0, 13, 128, 13);
        u8g2.setFont(u8g2_font_6x10_tf);

        wifi_mode_t mode = WiFi.getMode();
        if (mode == WIFI_AP || mode == WIFI_AP_STA) {
            u8g2.drawStr(0, 16, "Mode: WiFi Setup");
            String apssid = String(AP_SSID);
            if (apssid.length() > 18) apssid = apssid.substring(0, 16) + "..";
            u8g2.drawStr(0, 26, apssid.c_str());
            u8g2.drawStr(0, 36, ("IP: " + WiFi.softAPIP().toString()).c_str());
            u8g2.drawStr(0, 46, "Open /wifi-setup");
        } else if (WiFi.status() == WL_CONNECTED) {
            u8g2.drawStr(0, 16, "WiFi: Connected");
            String ssid = WiFi.SSID();
            if (ssid.length() > 16) ssid = ssid.substring(0, 14) + "..";
            u8g2.drawStr(0, 26, ("SSID: " + ssid).c_str());
            u8g2.drawStr(0, 36, ("IP: " + WiFi.localIP().toString()).c_str());
            int rssi = WiFi.RSSI();
            String sig = rssi > -50 ? "Excellent" : rssi > -60 ? "Good" : rssi > -70 ? "Fair" : "Weak";
            u8g2.drawStr(0, 46, ("Signal: " + sig).c_str());
            u8g2.drawStr(0, 56, ("Host: " + String(HOSTNAME)).c_str());
        } else {
            u8g2.drawStr(0, 26, "WiFi: Connecting...");
            u8g2.drawStr(0, 46, "Standalone mode");
        }
        u8g2.sendBuffer();
    }

private:
    void showMainScreen(float boilerSupplyTemp, float floorSupplyTemp, float burningTemp, float ambientLowTemp,
                        bool boilerPump, bool heatingPump, bool fans, float targetBurningTemp, int airIntakePosition) {
        u8g2.setFont(u8g2_font_7x13_tf);
        u8g2.drawStr(0, 0, "Biomass Boiler");
        u8g2.drawLine(0, 13, 128, 13);
        u8g2.setFont(u8g2_font_6x10_tf);

        char buf[32];
        sprintf(buf, "Cal:   %.1fC", boilerSupplyTemp); u8g2.drawStr(0,  16, buf);
        sprintf(buf, "Suelo: %.1fC", floorSupplyTemp);  u8g2.drawStr(0,  26, buf);
        sprintf(buf, "Burn:  %.1fC", burningTemp);      u8g2.drawStr(0,  36, buf);
        sprintf(buf, "Amb:   %.1fC", ambientLowTemp);   u8g2.drawStr(0,  46, buf);

        u8g2.drawStr(70, 16, boilerPump  ? "BPump:ON"  : "BPump:OFF");
        u8g2.drawStr(70, 26, heatingPump ? "HPump:ON"  : "HPump:OFF");
        u8g2.drawStr(70, 36, fans        ? "Fans:ON"   : "Fans:OFF");

        sprintf(buf, "Tgt:%.1fC Air:%d%%", targetBurningTemp, airIntakePosition);
        u8g2.drawStr(0, 56, buf);
    }

    _U8G2_TYPE u8g2;
    bool available;
    ScreenType currentScreen;
    unsigned long lastScreenToggle;
    unsigned long screenToggleInterval;
};

#endif // DISPLAY_H
