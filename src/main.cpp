#include <Arduino.h>
#include <WiFi.h>
#include <Esp.h>
#include <esp_wifi.h>
#include "time.h"
#include "camera.h"
#include "utils.h"
#include "secrets.h"
#include "net.h"
#include <time.h> 
#include "storage.h"


const uint8_t garageOpenerPin = 14;


const char *WiFiSSID = SECRET_WIFI_SSID;
const char *WiFiPass = SECRET_WIFI_PASS;
const uint32_t cameraUpdatePeriodMs = 2000;
const uint32_t howLongBeforeRestartIfNotConnecting = 60000;


const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const char* ntpServer3 = "time.windows.com";
const char* timeZone = "MST7MDT,M3.2.0,M11.1.0";//https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

StorageData storageData;


Net NetClient(SECRET_DEVICE_NAME, SECRET_ENCROKEY, SECRET_HOST_ADDRESS, SECRET_HOST_PORT);


void packetReceived(uint8_t* data, uint32_t dataLength){
    Serial.print("NetClient recieved:");
    Serial.println(String(data, dataLength));
    switch (data[0]){
        case 1:
            digitalWrite(garageOpenerPin, HIGH);
            delay(250);
            digitalWrite(garageOpenerPin, LOW);
        break;
    }
}

void onConnected(){
    Serial.println("NetClient Connected");
}

void onDisconnected(){
    Serial.println("NetClient disconnected");
}

void setup(){
    //Setup serial comm
    Serial.begin(115200);
    Serial.println("Initializing...");


    //Setup time
    configTime(0, 0, ntpServer1, ntpServer2, ntpServer3);  // 0, 0 because we will use TZ in the next line
    setenv("TZ", timeZone, 1);            // Set environment variable with your time zone
    tzset();

    //Setup non volatile storage
    StorageData defaultStorage;
    initStorage(&defaultStorage, storageData);

    //Setup IO
    pinMode(garageOpenerPin, OUTPUT);
    digitalWrite(garageOpenerPin, LOW);

    //Setup WiFi
    WiFi.setMinSecurity(WIFI_AUTH_OPEN);
    WiFi.mode(WIFI_STA);
    uint8_t newMac[6]={0,0,0,0,0,0};
    esp_fill_random(newMac+1, 5);
    esp_wifi_set_mac(WIFI_IF_STA, newMac);
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.begin(WiFiSSID, WiFiPass);

    //Setup NetClient
    NetClient.setPacketReceivedCallback(&packetReceived);
    NetClient.setOnConnected(&onConnected);
    NetClient.setOnDisconnected(&onDisconnected);
}


void loop(){
    static uint32_t lastConnectTime=0;
    static uint8_t failReconnects=0;

    static uint32_t lastCameraTime=0;
    static uint32_t lastReadyTime=0;


    uint32_t currentTime = millis();

    if (WiFi.status() != WL_CONNECTED){//Reconnect to WiFi
        if (isTimeToExecute(lastConnectTime, 2000)){
            Serial.println("Waiting for autoreconnect...");
            failReconnects++;
            if (failReconnects>10){
                Serial.println("Autoreconnect failed, generating new MAC and retrying...");
                failReconnects=0;
                WiFi.disconnect();
                uint8_t newMac[6]={0,0,0,0,0,0};
                esp_fill_random(newMac+1, 5);
                esp_wifi_set_mac(WIFI_IF_STA, newMac);
                WiFi.begin(WiFiSSID, WiFiPass);
            }
        }
    }else{
        failReconnects=0;
    }

    if (NetClient.loop()){
        lastReadyTime=currentTime;
        if (isTimeToExecute(lastCameraTime, cameraUpdatePeriodMs)){
            CAMERA_CAPTURE capture;

            if (cameraCapture(capture)){
                NetClient.sendBinary(capture.jpgBuff, capture.jpgBuffLen);
                cameraCaptureCleanup(capture);
            }else{
                Serial.println("failed to capture ");
            }
        }
    }else{
        if (currentTime-lastReadyTime > howLongBeforeRestartIfNotConnecting || lastReadyTime>currentTime){//Crude edge case handling, if overflow, just restart
            ESP.restart();
        }
    }
}