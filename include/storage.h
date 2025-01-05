#pragma once
#include <stdint.h>
#include "esp_camera.h"


typedef struct {
    uint32_t placeHolder;
    framesize_t frame_size;
    int quality;
} StorageData;

const uint32_t INITIALIZED_VALUE = 0x0BEDBEEF + sizeof(StorageData);

bool initStorage(StorageData* defaultStorageData, StorageData& storageData);//returns true if memory was already initialized
void commitStorage(StorageData& storageData);