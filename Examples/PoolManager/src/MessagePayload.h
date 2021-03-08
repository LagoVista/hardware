#ifndef MESSAGEPAYLOAD_H
#define MESSAGEPAYLOAD_H

#include <Arduino.h>
#include <ArduinoJson.h>

class PoolStatusMessagePayload {
    public:
    int countsIn;
    int countsOut;
    int rssi;

    double poolTemperature;
    double heatedTemperature;

    double poolSetpoint;
    double spaSetpoint;

    String mode;

    String compressor;
    String fan;
    String version;

    String flow;
    String highPressure;
    String lowPressure;

    String getSON() {
        const size_t capacity = JSON_OBJECT_SIZE(17);
        DynamicJsonDocument doc(capacity);

        doc["countsIn"] = countsIn;
        doc["countsOut"] = countsOut;
        doc["poolTemperature"] = poolTemperature;        
        doc["heatedTemperature"] = heatedTemperature;
        doc["poolSetpoint"] = poolSetpoint;
        doc["spaSetpoint"] = spaSetpoint;
        doc["mode"] = mode;
        doc["rssi"] = rssi;
        doc["compressor"] = compressor;
        doc["fan"] = fan;
        doc["flow"] = flow;
        doc["version"] = version;
        doc["highPressure"] = highPressure;
        doc["lowPressure"] = lowPressure;

        String output;
        serializeJson(doc, output);

        return output;
    }
};

#endif