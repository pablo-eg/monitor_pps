#include "STM32SimpleRTC.h"

hourAM_PM_t STM32SimpleRTC::_hoursPeriod = HOUR_AM;
uint8_t     STM32SimpleRTC::_hours = 0;
uint8_t     STM32SimpleRTC::_minutes = 0;
uint8_t     STM32SimpleRTC::_seconds = 0;
uint32_t    STM32SimpleRTC::_subSeconds = 0;
uint8_t     STM32SimpleRTC::_year = 0;
uint8_t     STM32SimpleRTC::_month = 0;
uint8_t     STM32SimpleRTC::_day = 0;
uint8_t     STM32SimpleRTC::_wday = 0;

void STM32SimpleRTC::begin(bool _reset)
{
	RTC_init(HOUR_FORMAT_24, sourceClock_t::LSI_CLOCK, _reset);

	// Must be set before call of sync methods
	syncDateTime();
}

void STM32SimpleRTC::setAlarmInSeconds(uint16_t tseconds)
{
	syncDateTime();

	addSecondsToTime(tseconds);
	enableAlarm(MATCH_HHMMSS); //Does not use day
}

void STM32SimpleRTC::addSecondsToTime(uint16_t tseconds)
{
	if (tseconds >= 3600)
		tseconds = 3599;
	uint16_t sum = tseconds + _seconds;
	_seconds = sum % 60;
	sum /= 60; //convert sum to minutes
	sum += _minutes;
	_minutes = sum % 60;
	sum /= 60; //convert sum to hours
	sum += _hours;
	_hours = sum % 24;

}

/**
  * @brief  synchronise the time from the current RTC one
  * @param  none
  */
void STM32SimpleRTC::syncDateTime(void)
{
	_hoursPeriod = HOUR_AM;
	RTC_GetTime(&_hours, &_minutes, &_seconds, &_subSeconds, &_hoursPeriod);
	RTC_GetDate(&_year, &_month, &_day, &_wday);

}

void STM32SimpleRTC::enableAlarm(Alarm_Match match)
{
	switch (match)
	{
	case MATCH_OFF:
		RTC_StopAlarm();
		break;
	case MATCH_DHHMMSS:
	case MATCH_HHMMSS:
	case MATCH_MMSS:
	case MATCH_SS:
		RTC_StartAlarm(_day, _hours, _minutes, _seconds, _subSeconds, _hoursPeriod, static_cast<uint8_t>(match));
		break;
	default:
		break;
	}
}

void STM32SimpleRTC::stopAlarm()
{
	RTC_StopAlarm();
}

uint32_t STM32SimpleRTC::getTimeInSeconds()
{
	return _seconds + (_minutes + (_hours + _day * 24) * 60) * 60;
}