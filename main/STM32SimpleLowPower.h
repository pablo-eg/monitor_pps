#pragma once
#include <Arduino.h>

// Check if PWR HAL enable in variants/board_name/stm32yzxx_hal_conf.h
#ifndef HAL_PWR_MODULE_ENABLED
#error "PWR configuration is missing. Check flag HAL_PWR_MODULE_ENABLED in variants/board_name/stm32yzxx_hal_conf.h"
#endif

class STM32SimpleLowPower
{
public:
	static void begin();
	static void deepSleep(uint32_t seconds);
private:
	static serial_t *_serial;    // Serial for wakeup from deep sleep

};