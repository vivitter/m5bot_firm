#ifndef WIFI_HARDWARE_H
#define WIFI_HARDWARE_H

#include <WiFi.h>
#include "wifi_setting.h"

WiFiClient client;
IPAddress server(ROS_MASTER_ADDR);
class WiFiHardware {
    public:
    WiFiHardware() {};
    void init() {
        client.connect(server, 11411);
    }
    int read() {
        return client.read();
    }
    void write(uint8_t* data, int length) {
        client.write(data, length);
    }
    unsigned long time() {
        return millis(); // easy; did this one for you
    }
};

#endif // !WIFI_HARDWARE_H