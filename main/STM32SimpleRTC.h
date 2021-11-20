#pragma once
#include <Arduino.h>
#include "rtc.h"
#include "low_power.h"
#include "clock.h"

// Check if RTC HAL enable in variants/board_name/stm32yzxx_hal_conf.h
#ifndef HAL_RTC_MODULE_ENABLED
#error "RTC configuration is missing. Check flag HAL_RTC_MODULE_ENABLED in variants/board_name/stm32yzxx_hal_conf.h"
#endif

//#define DEBUG_STM32SIMPLERTC


class STM32SimpleRTC
{
public:
	enum Alarm_Match : uint8_t
	{
		MATCH_OFF = OFF_MSK,                          // Never
		MATCH_SS = SS_MSK,                           // Every Minute
		MATCH_MMSS = SS_MSK | MM_MSK,                  // Every Hour
		MATCH_HHMMSS = SS_MSK | MM_MSK | HH_MSK,         // Every Day
		MATCH_DHHMMSS = SS_MSK | MM_MSK | HH_MSK | D_MSK, // Every Month
	};

	static void begin(bool _reset = true);
	static void setAlarmInSeconds(uint16_t tseconds);
	static void stopAlarm();
	static uint32_t getTimeInSeconds();

private:
	static hourAM_PM_t _hoursPeriod;
	static uint8_t     _hours;
	static uint8_t     _minutes;
	static uint8_t     _seconds;
	static uint32_t    _subSeconds;
	static uint8_t     _year;
	static uint8_t     _month;
	static uint8_t     _day;
	static uint8_t     _wday;

	static void syncDateTime(void);
	static void addSecondsToTime(uint16_t tseconds);
	static void enableAlarm(Alarm_Match match);
};