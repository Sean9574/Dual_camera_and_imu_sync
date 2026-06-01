// CameraManager.h
// Manages synchronized camera trigger pulses via GPIO
// Supports one or more USB cameras with external trigger input
// Typical OV9281 global shutter specs: 5-50 µs pulse width, rising edge trigger

#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <Arduino.h>

class CameraManager
{
public:
    CameraManager()
        : triggerPin(0), 
          pulseWidthMicros(20), 
          triggerIntervalMs(500),  // Default 0.5s = 2 Hz
          lastTriggerMs(0),
          enabled(false)
    {
    }

    // Initialize camera trigger on specified pin
    // pulseWidth_us: pulse duration in microseconds (typical 5-50 for OV9281)
    // triggerInterval_ms: time between triggers in milliseconds
    void begin(uint8_t pin, uint16_t pulseWidth_us = 20, uint32_t triggerInterval_ms = 500)
    {
        triggerPin = pin;
        pulseWidthMicros = pulseWidth_us;
        triggerIntervalMs = triggerInterval_ms;
        lastTriggerMs = millis();
        enabled = true;

        pinMode(triggerPin, OUTPUT);
        digitalWrite(triggerPin, LOW);  // Idle low

        Serial.print("CameraManager: Initialized on GPIO");
        Serial.print(pin);
        Serial.print(", pulse=");
        Serial.print(pulseWidth_us);
        Serial.print(" µs, interval=");
        Serial.print(triggerInterval_ms);
        Serial.println(" ms");
    }

    // Call in main loop to handle trigger timing
    // Returns true if trigger was sent, false otherwise
    bool update()
    {
        if (!enabled)
            return false;

        const uint32_t now_ms = millis();
        if ((now_ms - lastTriggerMs) >= triggerIntervalMs)
        {
            sendTrigger();
            lastTriggerMs = now_ms;
            return true;
        }
        return false;
    }

    // Manually send one trigger pulse (for testing)
    void sendTrigger()
    {
        // Rising edge trigger: LOW → HIGH → LOW
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(pulseWidthMicros);
        digitalWrite(triggerPin, LOW);
    }

    // Get current trigger interval (milliseconds)
    uint32_t getTriggerInterval() const { return triggerIntervalMs; }

    // Set trigger interval dynamically (milliseconds)
    // Rate must be > 0
    void setTriggerInterval(uint32_t intervalMs)
    {
        if (intervalMs > 0)
        {
            triggerIntervalMs = intervalMs;
            float hz = 1000.0f / intervalMs;
            Serial.print("CameraManager: Trigger rate set to ");
            Serial.print(hz, 1);
            Serial.println(" Hz");
        }
    }

    // Get current pulse width (microseconds)
    uint16_t getPulseWidth() const { return pulseWidthMicros; }

    // Set pulse width dynamically (microseconds)
    // Typical 5-50 for OV9281
    void setPulseWidth(uint16_t widthUs)
    {
        if (widthUs > 0 && widthUs <= 1000)  // Sanity check: max 1 ms
        {
            pulseWidthMicros = widthUs;
            Serial.print("CameraManager: Pulse width set to ");
            Serial.print(widthUs);
            Serial.println(" µs");
        }
    }

    bool isEnabled() const { return enabled; }
    void enable() { enabled = true; }
    void disable() { enabled = false; }

private:
    uint8_t triggerPin;
    uint16_t pulseWidthMicros;      // Pulse duration (µs)
    uint32_t triggerIntervalMs;     // Time between pulses (ms)
    uint32_t lastTriggerMs;         // Timestamp of last trigger
    bool enabled;
};

#endif // CAMERA_MANAGER_H
