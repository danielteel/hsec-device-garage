#pragma once
#include <WiFiClient.h>

enum NETSTATUS {
    NOTHING,
    INITIAL_SENT,
    READY
};

enum RECVSTATE {
    LEN1,
    LEN2,
    LEN3,
    LEN4,
    PAYLOAD
};


class Net {
    public:
        Net(String deviceName, String encroKey, String address, uint16_t port);
        ~Net();
        bool loop();

        bool sendString(String str);
        bool sendBinary(uint8_t* data, uint32_t dataLen);
        
        bool ready();
        
        void setPacketReceivedCallback(void (*packetReceivedCallback)(uint8_t*, uint32_t));
        void setOnConnected(void (*onConnected)(void));
        void setOnDisconnected(void (*onDisconnected)(void));

    private:
        WiFiClient Client;
        String deviceName;
        uint8_t encroKey[32];
        String hostAddress;
        uint16_t port;

        uint32_t clientsHandshake;
        uint32_t serversHandshake;

        NETSTATUS netStatus;
        RECVSTATE recvState;
        uint32_t packetLength;
        uint8_t* packetPayload=nullptr;
        uint32_t payloadRecvdCount=0;

        const uint32_t connectAttemptInterval=2000;
        uint32_t lastConnectAttempt=0;

        bool wasConnected=false;

        void (*packetReceived)(uint8_t* data, uint32_t dataLength)=nullptr;
        void (*onConnected)()=nullptr;
        void (*onDisconnected)()=nullptr;

    private:
        void errorOccured(String errorText);
        void attemptToConnect();
        bool sendPacket(uint8_t* data, uint32_t dataLength);
        void byteReceived(uint8_t data);
        void packetRecieved(uint32_t recvdHandshake, uint8_t* data, uint32_t dataLength);
        void processIncoming();
};