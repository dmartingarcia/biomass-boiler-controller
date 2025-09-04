#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <Arduino.h>
#include <CircularBuffer.hpp>
#include "config.h"

class LogBuffer {
public:
    LogBuffer() {}

    void begin() {
        // No specific initialization needed
    }

    // Add a message to the buffer
    void log(const String& message) {
        // Create an entry with timestamp
        char timestamp[20];
        unsigned long now = millis();
        sprintf(timestamp, "[%10lu] ", now);
        String entry = String(timestamp) + message;

        // Add to circular buffer
        buffer.push(entry);

        // Also send to Serial
        Serial.println(entry);
    }

    // Get all messages from the buffer as a single string
    String getAll() {
        String result = "";

        for (int i = 0; i < buffer.size(); i++) {
            result += buffer[i] + "\n";
        }

        return result;
    }

    // Get the last N messages as a single string
    String getLast(int count) {
        String result = "";
        int start = max(0, buffer.size() - count);

        for (int i = start; i < buffer.size(); i++) {
            result += buffer[i] + "\n";
        }

        return result;
    }

    // Clear the buffer
    void clear() {
        buffer.clear();
    }

private:
    CircularBuffer<String, LOG_BUFFER_SIZE> buffer;
};

#endif // LOG_BUFFER_H
