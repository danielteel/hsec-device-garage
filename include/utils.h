#pragma once

#include <Arduino.h>

bool isTimeToExecute(uint32_t &lastTime, uint32_t interval);