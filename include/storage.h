#pragma once

#include <stdint.h>


typedef struct {
    uint32_t placeHolder;
} StorageData;

const uint32_t INITIALIZED_VALUE = 0x0BADBEEF + sizeof(StorageData);

bool initStorage(StorageData* defaultStorageData, StorageData& storageData);//returns true if memory was already initialized
void commitStorage(StorageData& storageData);