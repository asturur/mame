// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi
#ifndef MAME_NOKIA_NGAGE_MMC_H
#define MAME_NOKIA_NGAGE_MMC_H

#pragma once

#include "machine/mmc_card.h"

class ngage_mmc_device : public mmc_card_device
{
public:
	ngage_mmc_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	void set_hot_swap(bool enabled) { m_hot_swap = enabled; }
	auto event_cb() { return m_event_cb.bind(); }
	auto media_change_cb() { return m_media_change_cb.bind(); }
	u32 read(offs_t offset, u32 mem_mask);
	void write(offs_t offset, u32 data, u32 mem_mask);
	u32 irq_sources() const;
	bool dma_ready(bool write) const;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;
	virtual bool is_reset_on_load() const noexcept override;
	virtual void media_unloaded() override;

private:
	enum : u8 { FIFO_IDLE, FIFO_RX, FIFO_TX };
	static constexpr u32 DATA_BUSY = 0x004, CMD_BUSY = 0x008, RESP_TIMEOUT = 0x040;
	static constexpr u32 DATA_TIMEOUT = 0x080, RESP_READY = 0x100, DATA_ERROR = 0x400;

	void reset_host();
	void abort_card_transfer();
	void start(u32 control);
	void complete_card_command();
	void response(u32 value);
	void response(u8 const *bytes);
	void begin_data(bool write);
	void end_data();
	u32 response_r();
	u32 fifo_r(u32 mask);
	void fifo_w(u32 data, u32 mask);
	TIMER_CALLBACK_MEMBER(command_complete);
	TIMER_CALLBACK_MEMBER(command_tail);
	TIMER_CALLBACK_MEMBER(notify_media_change);

	devcb_write_line m_event_cb;
	devcb_write_line m_media_change_cb;
	emu_timer *m_command_timer;
	emu_timer *m_tail_timer;

	bool m_hot_swap;
	u32 m_reg[16];
	u32 m_status;
	u8 m_type;
	u8 m_command;
	u32 m_argument;
	u32 m_response[4];
	u8 m_response_words;
	u8 m_response_pos;

	u8 m_fifo_direction;
	u8 m_fifo[4096];
	u16 m_fifo_length;
	u16 m_fifo_pos;
};

DECLARE_DEVICE_TYPE(NGAGE_MMC, ngage_mmc_device)

#endif // MAME_NOKIA_NGAGE_MMC_H
