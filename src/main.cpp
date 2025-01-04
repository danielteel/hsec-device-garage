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
#include "DHTesp.h"


const uint8_t dht11Pin = 13;
const uint8_t garageOpenerPin = 14;

DHTesp dht;

const char *WiFiSSID = SECRET_WIFI_SSID;
const char *WiFiPass = SECRET_WIFI_PASS;

const uint32_t sendPeriod = 1000;//How often a thing should be sent, and then the state machine advanced
const uint32_t howLongBeforeRestartIfNotConnecting = 300000;//restart esp32 if havent been able to connect to server for 5 minutes


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
    NetClient.sendString("i=Operate:void:1");
}

void onDisconnected(){
    Serial.println("NetClient disconnected");
}

void WiFiSetup(bool doRandomMAC){
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    uint8_t newMac[6]={0,0,0,0,0,0};
    String newHostName=SECRET_DEVICE_NAME;
    if (doRandomMAC){
        esp_fill_random(newMac+1, 5);
        for (int i=0;i<6;i++){
            newHostName+=String(newMac[i], 16);
        }
    }
    WiFi.setHostname(newHostName.c_str());

    WiFi.mode(WIFI_STA);
    if (doRandomMAC) esp_wifi_set_mac(WIFI_IF_STA, newMac);
    WiFi.setMinSecurity(WIFI_AUTH_OPEN);
    WiFi.setSleep(WIFI_PS_NONE);

    WiFi.begin(WiFiSSID, WiFiPass);
    Serial.println("WiFiSetup:");
    Serial.print("    Mac Address:");
    Serial.println(WiFi.macAddress());
    Serial.print("    Hostname:");
    Serial.println(newHostName);
}

void setup(){
    //Setup serial comm
    Serial.begin(115200);
    Serial.println("Initializing...");

    //Setup camera
    cameraSetup();

    //Setup DHT11
    dht.setup(dht11Pin, DHTesp::DHT22);

    //Setup time
    configTime(0, 0, ntpServer1, ntpServer2, ntpServer3);
    setenv("TZ", timeZone, 1);
    tzset();

    //Setup non volatile storage
    StorageData defaultStorage;
    initStorage(&defaultStorage, storageData);

    //Setup IO
    pinMode(garageOpenerPin, OUTPUT);
    digitalWrite(garageOpenerPin, LOW);

    //Setup WiFi
    WiFiSetup(false);

    //Setup NetClient
    NetClient.setPacketReceivedCallback(&packetReceived);
    NetClient.setOnConnected(&onConnected);
    NetClient.setOnDisconnected(&onDisconnected);
}


class StateMachine {
    public:
        StateMachine(uint8_t max){
            maxStates=max;
        }

        uint8_t maxStates=0;
        uint8_t state=0;


        void next(){
            state++;
            if (state>=maxStates){
                state=0;
            }
        }
};

void loop(){
    static StateMachine sendState(2);

    static uint32_t lastConnectTime=0;
    static uint8_t failReconnects=0;

    static uint32_t lastSendTime=0;
    static uint32_t lastReadyTime=0;

    uint32_t currentTime = millis();

    if (WiFi.status() != WL_CONNECTED){//Reconnect to WiFi
        if (isTimeToExecute(lastConnectTime, 2000)){
            Serial.println("Waiting for autoreconnect...");
            failReconnects++;
            if (failReconnects>30){
                Serial.println("Autoreconnect failed, generating new MAC and retrying...");
                failReconnects=0;
                WiFiSetup(true);
            }
        }
    }else{
        failReconnects=0;
        if (NetClient.loop()){
            lastReadyTime=currentTime;

            if (isTimeToExecute(lastSendTime, sendPeriod)){
                switch (sendState.state){
                    case 0:                
                        CAMERA_CAPTURE capture;
                        if (cameraCapture(capture)){
                            NetClient.sendBinary(capture.jpgBuff, capture.jpgBuffLen);
                            cameraCaptureCleanup(capture);
                        }else{
                            Serial.println("failed to capture ");
                        }
                        break;
                    case 1:
                        float humidity = dht.getHumidity();
                        float temperature = dht.getTemperature();
                        String weather=String("w=humidity:")+String(humidity)+String(",temperature:")+String(temperature);
                        NetClient.sendString(weather);
                        break;
                }
                sendState.next();
            }
        }
    }

    if (currentTime-lastReadyTime > howLongBeforeRestartIfNotConnecting || lastReadyTime>currentTime){//Crude edge case handling, if overflow, just restart
        ESP.restart();
    }
}