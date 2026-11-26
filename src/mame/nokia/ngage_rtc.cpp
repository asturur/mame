// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi
// Battery-backed UEM counter. The absolute calendar base belongs to NOS PMM.

#include "emu.h"
#include "ngage_rtc.h"

#include "emuopts.h"

#include "multibyte.h"

#include <cstring>
#include <ctime>
#include <string_view>

DEFINE_DEVICE_TYPE(NGAGE_RTC, ngage_rtc_device, "ngage_rtc", "N-Gage UEM RTC")

ngage_rtc_device::ngage_rtc_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, NGAGE_RTC, tag, owner, clock)
	, device_nvram_interface(mconfig, *this)
{
}

void ngage_rtc_device::device_start()
{
	save_item(NAME(m_base));
}

u32 ngage_rtc_device::time_of_day(system_time const &time)
{
	return time.local_time.hour * 3600 + time.local_time.minute * 60 + time.local_time.second;
}

u32 ngage_rtc_device::counter() const
{
	return (m_base + u64(machine().time().seconds())) % COUNTER_PERIOD;
}

void ngage_rtc_device::set_counter(u32 value)
{
	m_base = (value + COUNTER_PERIOD - u64(machine().time().seconds()) % COUNTER_PERIOD) % COUNTER_PERIOD;
}

void ngage_rtc_device::set_day(u8 day)
{
	set_counter(day * DAY_SECONDS + counter() % DAY_SECONDS);
}

void ngage_rtc_device::nvram_default()
{
	system_time time;
	machine().base_datetime(time);
	set_counter(time_of_day(time));
}

bool ngage_rtc_device::nvram_read(util::read_stream &file)
{
	u8 data[16];
	auto const [err, actual] = read(file, data, sizeof(data));
	if (err || actual != sizeof(data) || std::memcmp(data, "NRTC", 4) || get_u32le(&data[4]) >= COUNTER_PERIOD)
	{
		nvram_default();
		return false;
	}

	system_time base;
	machine().base_datetime(base);
	u32 const saved_counter = get_u32le(&data[4]);
	std::string_view const rtc_override = machine().options().rtc_time();
	if (!*machine().options().playback() && rtc_override.size() == 14
			&& rtc_override.find_first_not_of("0123456789") == std::string_view::npos)
	{
		// NOS keeps the absolute date in PMM. Override the UEM time of day.
		set_counter(saved_counter - (saved_counter % DAY_SECONDS) + time_of_day(base));
		return true;
	}

	u64 const saved = get_u64le(&data[8]);
	s64 const now = base.time;
	u64 const elapsed = now > 0 && saved && u64(now) > saved ? u64(now) - saved : 0;
	set_counter((saved_counter + elapsed % COUNTER_PERIOD) % COUNTER_PERIOD);
	return true;
}

bool ngage_rtc_device::nvram_write(util::write_stream &file)
{
	u8 data[16] = { 'N', 'R', 'T', 'C' };
	put_u32le(&data[4], counter());
	s64 const now = std::time(nullptr);
	put_u64le(&data[8], now > 0 ? now : 0);
	auto const [err, actual] = write(file, data, sizeof(data));
	return !err && actual == sizeof(data);
}
