#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "SharedData.h"

class WebDashboard {
private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    const char* ssid;
    const char* pass;
    static const char index_html[] PROGMEM;
    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

public:
    WebDashboard(const char* ssid, const char* pass);
    void begin();
    void notifyClients(); 
    void cleanup();       
};

#endif