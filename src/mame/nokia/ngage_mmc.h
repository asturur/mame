// license:GPL-2.0+
// copyright-holders:Andrea Bogazzi
#ifndef MAME_NOKIA_NGAGE_MMC_H
#define MAME_NOKIA_NGAGE_MMC_H

#pragma once

class ngage_mmc_device : public device_t, public device_image_interface
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
	bool mounted() const { return m_payload_size != 0; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual void call_unload() override;
	virtual bool is_readable() const noexcept override;
	virtual bool is_writeable() const noexcept override;
	virtual bool is_creatable() const noexcept override;
	virtual bool is_reset_on_load() const noexcept override;
	virtual char const *image_type_name() const noexcept override;
	virtual char const *image_brief_type_name() const noexcept override;
	virtual char const *file_extensions() const noexcept override;

private:
	enum : u8 { IDLE, READY, IDENT, STANDBY, TRANSFER, DATA, RECEIVE, INACTIVE };
	enum : u8 { FIFO_IDLE, FIFO_RX, FIFO_TX };
	static constexpr u32 DATA_BUSY = 0x004, CMD_BUSY = 0x008, RESP_TIMEOUT = 0x040;
	static constexpr u32 DATA_TIMEOUT = 0x080, RESP_READY = 0x100, DATA_ERROR = 0x400;

	void reset_card();
	void abort_card_transfer();
	void start(u32 control);
	void card_command();
	void response(u32 value);
	void response(u8 const *bytes);
	void begin_data(bool write);
	void end_data();
	u32 response_r();
	u32 fifo_r(u32 mask);
	void fifo_w(u32 data, u32 mask);
	bool in_range(u64 offset, u32 length) const;
	TIMER_CALLBACK_MEMBER(command_complete);
	TIMER_CALLBACK_MEMBER(command_tail);
	TIMER_CALLBACK_MEMBER(notify_media_change);

	devcb_write_line m_event_cb;
	devcb_write_line m_media_change_cb;
	emu_timer *m_command_timer;
	emu_timer *m_tail_timer;

	bool m_hot_swap;
	u64 m_mount_id;
	u64 m_saved_mount_id;
	u64 m_payload_size;
	u64 m_card_capacity;
	u8 m_cid[16];
	u8 m_csd[16];
	u32 m_ocr;
	bool m_write_protected;

	u32 m_reg[16];
	u32 m_status;
	u8 m_type;
	u8 m_command;
	u32 m_argument;
	u32 m_response[4];
	u8 m_response_words;
	u8 m_response_pos;
	u8 m_card_state;
	u16 m_rca;
	u16 m_block_length;
	bool m_multiblock;
	u64 m_next_address;

	u8 m_fifo_direction;
	u8 m_fifo[4096];
	u16 m_fifo_length;
	u16 m_fifo_pos;
};

DECLARE_DEVICE_TYPE(NGAGE_MMC, ngage_mmc_device)

#endif // MAME_NOKIA_NGAGE_MMC_H
