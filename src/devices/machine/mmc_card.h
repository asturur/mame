// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi
#ifndef MAME_MACHINE_MMC_CARD_H
#define MAME_MACHINE_MMC_CARD_H

#pragma once

class mmc_card_device : public device_t, public device_image_interface
{
protected:
	mmc_card_device(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock);

	enum class reply_type { none, short_response, long_response, timeout, no_media, illegal };
	enum class transfer_type { none, read, write, error };
	enum class data_result { ok, timeout, io_error };
	struct command_reply
	{
		reply_type type = reply_type::none;
		u32 value = 0;
		u8 const *register_data = nullptr;
		transfer_type transfer = transfer_type::none;
	};

	bool mounted() const { return m_payload_size != 0; }
	bool revalidate_media();
	u32 block_length() const { return m_block_length; }
	command_reply card_command(u8 command, u32 argument);
	data_result begin_transfer(bool write, u8 *buffer, u32 length);
	bool finish_transfer(bool write, u8 const *buffer, u32 length);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual void call_unload() override;
	virtual bool is_readable() const noexcept override;
	virtual bool is_writeable() const noexcept override;
	virtual bool is_creatable() const noexcept override;
	virtual char const *image_type_name() const noexcept override;
	virtual char const *image_brief_type_name() const noexcept override;
	virtual char const *file_extensions() const noexcept override;
	virtual void media_unloaded() = 0;

private:
	static constexpr u32 SECTOR_BYTES = 512;
	static constexpr u32 OUT_OF_RANGE = 1U << 31;
	static constexpr u32 ADDRESS_ERROR = 1U << 30;
	static constexpr u32 BLOCK_LEN_ERROR = 1U << 29;
	static constexpr u32 WRITE_PROTECTED = 1U << 26;
	static constexpr u32 ILLEGAL_COMMAND = 1U << 22;

	enum : u8 { IDLE, READY, IDENT, STANDBY, TRANSFER, DATA, RECEIVE, INACTIVE };

	void reset_protocol();
	bool in_range(u64 offset, u32 length) const;

	u64 m_mount_id = 0;
	u64 m_saved_mount_id = 0;
	u64 m_payload_size = 0;
	u64 m_card_capacity = 0;
	u8 m_cid[16]{};
	u8 m_csd[16]{};
	u32 m_ocr = 0;
	bool m_write_protected = false;

	u8 m_card_state = IDLE;
	u16 m_rca = 1;
	u16 m_block_length = SECTOR_BYTES;
	bool m_multiblock = false;
	u64 m_next_address = 0;
};

#endif // MAME_MACHINE_MMC_CARD_H
