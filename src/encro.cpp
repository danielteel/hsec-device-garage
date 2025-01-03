#include <iostream>
#include <stdio.h>  
#include <stdlib.h>
#include <time.h>  
#include <cstring>
#include <Arduino.h>

void buildKeyFromString(const char* keyString, uint8_t* key){
    char tempHex[] = { 0,0,0 };
    for (uint8_t i = 0; i < 64; i += 2) {
        tempHex[0] = keyString[i];
        tempHex[1] = keyString[i + 1];
        key[i >> 1] = strtoul(tempHex, nullptr, 16);
    }
}

uint32_t leftRotate32(uint32_t num, uint8_t places) {
    uint64_t temp = (uint64_t)num << places;
    temp |= (uint64_t)num >> (32 - places);
    return (uint32_t)(temp & 0xFFFFFFFF);
}
uint32_t rightRotate32(uint32_t num, uint8_t places) {
    uint64_t temp = (uint64_t)num >> places;
    temp |= (uint64_t)num << (32 - places);
    return (uint32_t)(temp & 0xFFFFFFFF);
}
uint8_t leftRotate8(uint8_t num, uint8_t places) {
    uint16_t temp = (uint16_t)num << places;
    temp |= (uint16_t)num >> (8 - places);
    return (uint8_t)(temp & 0xFF);
}
uint8_t rightRotate8(uint8_t num, uint8_t places) {
    uint16_t temp = (uint16_t)num >> places;
    temp |= (uint16_t)num << (8 - places);
    return (uint8_t)(temp & 0xFF);
}

uint8_t* frame(uint32_t handshake, const uint8_t* data, uint32_t length, uint32_t& framedLength) {
    if (!data && length){
        return nullptr;
    }
    if (length>0XFFFF0){
        return nullptr;
    }
    uint8_t paddingLength = 4;
    const uint8_t handshakeLength=4;
    const uint8_t sizeLength=4;
    uint8_t modLength = (sizeLength + handshakeLength + paddingLength + length) % 16;
    if (modLength) {
        paddingLength = 16 - modLength % 16 + 4;
    }
    framedLength = sizeLength + handshakeLength + paddingLength + length;
    uint8_t* framed = (uint8_t*)malloc(framedLength);
    if (!framed){
        return nullptr;
    }
    esp_fill_random(framed+sizeLength, paddingLength);
    *((uint32_t*)framed) = length+handshakeLength;
    memmove(framed+sizeLength+paddingLength, &handshake, handshakeLength);
    if (data && length) memmove(framed+sizeLength+paddingLength+handshakeLength, data, length);
    return framed;
}

uint8_t* deframe(uint8_t* data, uint32_t framedLength, uint32_t& dataLength, uint32_t& handshake, bool& error) {
    error=false;
    dataLength = (*((uint32_t*)data))-4;
    if (dataLength>framedLength-4 || dataLength>0x0FFFF0){
        error=true;
        return nullptr;
    }
    handshake=*((uint32_t*)(data+(framedLength - dataLength)-4));
    if (dataLength){
        uint8_t* deframed = data + (framedLength - dataLength);
        return deframed;
    }
    return nullptr;
}


uint8_t* encrypt(uint32_t handshake, const uint8_t* data, uint32_t dataLength, uint32_t& encryptedLength, const uint8_t* key) {
    uint8_t* buffer = frame(handshake, data, dataLength, encryptedLength);
    if (!buffer){
        return nullptr;
    }

    for (uint8_t k = 0; k < 32; k++) {
        for (uint32_t i = 0; i < encryptedLength - 7; i++) {

            uint32_t b4 = leftRotate32(buffer[i + 0] | buffer[i + 2] << 8 | buffer[i + 4] << 16 | buffer[i + 6] << 24, key[k] % 31);
            buffer[i + 0] = b4 & 0xFF;
            buffer[i + 2] = b4 >> 8 & 0xFF;
            buffer[i + 4] = b4 >> 16 & 0xFF;
            buffer[i + 6] = b4 >> 24 & 0xFF;

            b4 = leftRotate32(buffer[i + 1] | buffer[i + 3] << 8 | buffer[i + 5] << 16 | buffer[i + 7] << 24, key[k] % 31);
            buffer[i + 1] = b4 & 0xFF;
            buffer[i + 3] = b4 >> 8 & 0xFF;
            buffer[i + 5] = b4 >> 16 & 0xFF;
            buffer[i + 7] = b4 >> 24 & 0xFF;

            buffer[i + 0] ^= buffer[i + 1];
            buffer[i + 1] ^= buffer[i + 2];
            buffer[i + 2] ^= buffer[i + 3];
            buffer[i + 3] ^= buffer[i + 4];
            buffer[i + 4] ^= buffer[i + 5];
            buffer[i + 5] ^= buffer[i + 6];
            buffer[i + 6] ^= buffer[i + 7];
            buffer[i + 7] ^= buffer[i + 0];

            buffer[i + 0] ^= key[k];
            buffer[i + 1] ^= leftRotate8(key[k], 0 + buffer[i + 0] % 2);
            buffer[i + 2] ^= leftRotate8(key[k], 1 + buffer[i + 1] % 2);
            buffer[i + 3] ^= leftRotate8(key[k], 2 + buffer[i + 2] % 2);
            buffer[i + 4] ^= leftRotate8(key[k], 3 + buffer[i + 3] % 2);
            buffer[i + 5] ^= leftRotate8(key[k], 4 + buffer[i + 4] % 2);
            buffer[i + 6] ^= leftRotate8(key[k], 5 + buffer[i + 5] % 2);
            buffer[i + 7] ^= leftRotate8(key[k], 6 + buffer[i + 6] % 2);
        }
    }
    return buffer;
}


uint8_t* decrypt(uint32_t& handshake, uint8_t* data, uint32_t dataLength, uint32_t& decryptedLength, const uint8_t* key, bool& error) {
    error=false;

    for (uint8_t k = 31; k >= 0 && k != 0xFF; k--) {
        for (uint32_t i = dataLength - 8; i >= 0 && i != 0xFFFFFFFF; i--) {
            data[i + 7] ^= leftRotate8(key[k], 6 + data[i + 6] % 2);
            data[i + 6] ^= leftRotate8(key[k], 5 + data[i + 5] % 2);
            data[i + 5] ^= leftRotate8(key[k], 4 + data[i + 4] % 2);
            data[i + 4] ^= leftRotate8(key[k], 3 + data[i + 3] % 2);
            data[i + 3] ^= leftRotate8(key[k], 2 + data[i + 2] % 2);
            data[i + 2] ^= leftRotate8(key[k], 1 + data[i + 1] % 2);
            data[i + 1] ^= leftRotate8(key[k], 0 + data[i + 0] % 2);
            data[i + 0] ^= key[k];

            data[i + 7] ^= data[i + 0];
            data[i + 6] ^= data[i + 7];
            data[i + 5] ^= data[i + 6];
            data[i + 4] ^= data[i + 5];
            data[i + 3] ^= data[i + 4];
            data[i + 2] ^= data[i + 3];
            data[i + 1] ^= data[i + 2];
            data[i + 0] ^= data[i + 1];

            uint32_t b4 = rightRotate32(data[i + 0] | data[i + 2] << 8 | data[i + 4] << 16 | data[i + 6] << 24, key[k] % 31);
            data[i + 0] = b4 & 0xFF;
            data[i + 2] = b4 >> 8 & 0xFF;
            data[i + 4] = b4 >> 16 & 0xFF;
            data[i + 6] = b4 >> 24 & 0xFF;

            b4 = rightRotate32(data[i + 1] | data[i + 3] << 8 | data[i + 5] << 16 | data[i + 7] << 24, key[k] % 31);
            data[i + 1] = b4 & 0xFF;
            data[i + 3] = b4 >> 8 & 0xFF;
            data[i + 5] = b4 >> 16 & 0xFF;
            data[i + 7] = b4 >> 24 & 0xFF;
        }
    }
    uint8_t* decrypted = deframe(data, dataLength, decryptedLength, handshake, error);
    return decrypted;
}