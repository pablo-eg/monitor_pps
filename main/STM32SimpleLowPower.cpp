#include "STM32SimpleLowPower.h"
#include "STM32SimpleRTC.h"
#include "rtc.h"
#include "low_power.h"
#include "clock.h"

serial_t *STM32SimpleLowPower::_serial = NULL;

void STM32SimpleLowPower::begin()
{
	LowPower_init();
	STM32SimpleRTC::begin();
}

void STM32SimpleLowPower::deepSleep(uint32_t seconds)
{
	if (seconds > 0)
	{
		STM32SimpleRTC::setAlarmInSeconds(seconds);
	}
	LowPower_stop(_serial);
}

