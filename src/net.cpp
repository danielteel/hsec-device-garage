#include <Arduino.h>
#include <Esp.h>
#include "encro.h"
#include "net.h"
#include "utils.h"


Net::Net(String deviceName, String encroKeyString, String address, uint16_t port){
    this->deviceName=deviceName;
    this->hostAddress=address;
    this->port=port;

    buildKeyFromString(encroKeyString.c_str(), encroKey);
}

Net::~Net(){
    Client.stop();

    if (packetPayload){
        free(packetPayload);
        packetPayload=nullptr;
    }
}


void Net::errorOccured(String errorText){
    Client.stop();
    
    if (packetPayload){
        free(packetPayload);
        packetPayload=nullptr;
    }

    Serial.print("Net error occurred: ");
    Serial.println(errorText);
}

void Net::attemptToConnect(){
    if (!this->Client.connected()){
        if (wasConnected){
            wasConnected=false;
            if (onDisconnected) onDisconnected();
        }
        this->netStatus=NETSTATUS::NOTHING;
        this->recvState=RECVSTATE::LEN1;
        if (isTimeToExecute(this->lastConnectAttempt, connectAttemptInterval)){
            Serial.println("Attempting to connect to server...");
            if (this->Client.connect(this->hostAddress.c_str(), this->port)){
                delay(500);
                this->clientsHandshake=esp_random();

                this->Client.write((uint8_t)this->deviceName.length());
                this->Client.write(this->deviceName.c_str());
                if (sendPacket(nullptr, 0)){
                    this->netStatus=NETSTATUS::INITIAL_SENT;
                }else{
                    this->Client.stop();
                }
            }
        }
    }
}

void Net::setPacketReceivedCallback(void (*callback)(uint8_t*, uint32_t)){
    packetReceived=callback;
}

void Net::setOnConnected(void (*callback)(void)){
    onConnected=callback;
}
void Net::setOnDisconnected(void (*callback)(void)){
    onDisconnected=callback;
}

bool Net::sendString(String str){
    return sendPacket((uint8_t*)str.c_str(), str.length());
}

bool Net::sendBinary(uint8_t* data, uint32_t dataLength){
    return sendPacket(data, dataLength);
}

bool Net::ready(){
    return (netStatus==NETSTATUS::READY) && Client.connected();
}

bool Net::sendPacket(uint8_t* data, uint32_t dataLength){
    uint32_t encryptedLength;
    uint8_t* encrypted=encrypt(this->clientsHandshake, data, dataLength, encryptedLength, this->encroKey);
    if (encrypted){
        this->clientsHandshake++;
        this->Client.write((uint8_t*)&encryptedLength, 4);
        this->Client.write(encrypted, encryptedLength);
        free(encrypted);
        encrypted=nullptr;
        return true;
    }
    return false;
}

void Net::packetRecieved(uint32_t recvdHandshake, uint8_t* data, uint32_t dataLength){
    if (netStatus==NETSTATUS::INITIAL_SENT){
            serversHandshake=recvdHandshake+1;
            netStatus=NETSTATUS::READY;
            wasConnected=true;
            if (onConnected) onConnected();

    }else if (netStatus==NETSTATUS::READY){
        if (recvdHandshake==serversHandshake){
            serversHandshake++;
            if (packetReceived) packetReceived(data, dataLength);
        }else{
            //throw error, wrong handshake from expected
            String errorText="Wrong handshake, expected ";
            errorText+=String(serversHandshake)+" but recvd "+String(recvdHandshake);
            errorOccured(errorText);
            return;
        }
    }else{
        errorOccured("Unknown netStatus");
    }
}

void Net::byteReceived(uint8_t data){
    switch (recvState){
        case RECVSTATE::LEN1:
            packetLength=data;
            recvState=RECVSTATE::LEN2;
            break;
        case RECVSTATE::LEN2:
            packetLength|=(data<<8);
            recvState=RECVSTATE::LEN3;
            break;
        case RECVSTATE::LEN3:
            packetLength|=(data<<16);
            recvState=RECVSTATE::LEN4;
            break;
        case RECVSTATE::LEN4:
            packetLength|=(data<<24);
            payloadRecvdCount=0;
            if (packetPayload){
                free(packetPayload);
                packetPayload=nullptr;
            }
            if (packetLength==0){
                recvState=RECVSTATE::LEN1;
            }else{
                recvState=RECVSTATE::PAYLOAD;
                packetPayload=(uint8_t*)malloc(packetLength);
                if (!packetPayload){
                    errorOccured("Failed to allocate packet payload space");
                }
            }
            break;
        case RECVSTATE::PAYLOAD:
            packetPayload[payloadRecvdCount]=data;
            payloadRecvdCount++;
            if (payloadRecvdCount>=packetLength){
                recvState=RECVSTATE::LEN1;
                uint32_t recvdHandshake=0;
                uint32_t decryptedLength=0;
                bool errorOccurred=false;
                uint8_t* plainText=decrypt(recvdHandshake, packetPayload, packetLength, decryptedLength, encroKey, errorOccurred);

                if (errorOccurred){
                    errorOccured("Error occured decrypting payload");
                }else{
                    packetRecieved(recvdHandshake, plainText, decryptedLength);
                }
                if (packetPayload){
                    free(packetPayload);
                    packetPayload=nullptr;
                }
            }
            break;
        default:
            errorOccured("Unknown recvState");
            break;

    }
}

void Net::processIncoming(){
    while (this->Client.connected() && Client.available()>0){
        this->byteReceived(this->Client.read());
    }
}

bool Net::loop(){
    this->attemptToConnect();
    this->processIncoming();
    return ready();
}
