#include "utils.h"

bool isTimeToExecute(uint32_t &lastTime, uint32_t interval){
    uint32_t currentTime = millis();
    if ((currentTime - lastTime) >= interval || currentTime < lastTime){
        lastTime = currentTime;
        return true;
    }
    return false;
}
