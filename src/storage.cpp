#include <EEPROM.h>
#include "storage.h"

bool initStorage(StorageData* defaultStorageData, StorageData& storageData){
    bool wasAlreadyInitialized=true;
    EEPROM.begin(sizeof(StorageData)+sizeof(INITIALIZED_VALUE));
    if (EEPROM.readULong(0)!=INITIALIZED_VALUE){
        wasAlreadyInitialized=false;

        EEPROM.writeULong(0, INITIALIZED_VALUE);

        if (defaultStorageData){
            EEPROM.writeBytes(sizeof(INITIALIZED_VALUE), defaultStorageData, sizeof(StorageData));
        }else{
            for (uint32_t i=0; i<sizeof(StorageData); i++){
                EEPROM.writeByte(sizeof(INITIALIZED_VALUE)+i, 0);
            }
        }

        EEPROM.commit();
    }
    EEPROM.readBytes(sizeof(INITIALIZED_VALUE), &storageData, sizeof(StorageData));
    return wasAlreadyInitialized;
}

void commitStorage(StorageData& storageData){
        EEPROM.writeBytes(sizeof(INITIALIZED_VALUE), &storageData, sizeof(StorageData));
        EEPROM.commit();
}