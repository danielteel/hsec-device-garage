#pragma once
#include <stdint.h>

void buildKeyFromString(const char* keyString, uint8_t* key);
uint8_t* encrypt(uint32_t handshake, const uint8_t* data, uint32_t dataLength, uint32_t& encryptedLength, const uint8_t* key);
uint8_t* decrypt(uint32_t& handshake, uint8_t* data, uint32_t dataLength, uint32_t& decryptedLength, const uint8_t* key, bool& error);