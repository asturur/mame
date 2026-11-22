// license:GPL-2.0+
// copyright-holders:Andrea Bogazzi
#ifndef MAME_NOKIA_NGAGE_PCM_H
#define MAME_NOKIA_NGAGE_PCM_H

#pragma once

#include <span>

class ngage_pcm_device : public device_t, public device_sound_interface
{
public:
	ngage_pcm_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	auto reply_ready() { return m_reply_ready.bind(); }
	void set_memory(address_space &memory, u32 start, u32 end) ATTR_COLD;
	void disconnect();
	void receive(std::span<u8 const> packet);
	static constexpr u32 REPLY_WORDS = 7;
	std::span<u16 const> reply() const;
	void reply_consumed();

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	static constexpr u32 STREAM_RATE = 16'000;
	static constexpr u8 NO_HALF = 2;
	void set_route(std::span<u8 const> packet);
	void make_reply(u8 transaction, u8 opcode, u16 value = 0);
	void service_reply();
	void start_half(u8 preferred);
	void stop();
	TIMER_CALLBACK_MEMBER(half_complete);

	devcb_write_line m_reply_ready;
	address_space *m_memory;
	u32 m_memory_start;
	u32 m_memory_end;
	sound_stream *m_stream;
	emu_timer *m_completion;

	u16 m_reply[REPLY_WORDS];
	u8 m_media;
	u8 m_device;
	u8 m_object;
	u8 m_dsp_device;
	u8 m_dsp_object;
	bool m_reply_pending;
	bool m_open;
	bool m_first;
	bool m_final_done;
	u8 m_requested;
	u8 m_final;
	u8 m_playing;
	u8 m_free;
	u16 m_capacity;
	u16 m_final_frames;
	u32 m_address[2];
	u16 m_frames[2];
	u32 m_output_position;
	u8 m_repeat;
};

DECLARE_DEVICE_TYPE(NGAGE_PCM, ngage_pcm_device)

#endif // MAME_NOKIA_NGAGE_PCM_H
