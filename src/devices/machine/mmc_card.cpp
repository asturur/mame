// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi

#include "emu.h"
#include "mmc_card.h"

#include "multibyte.h"

#include <algorithm>
#include <cstring>
#include <random>

mmc_card_device::mmc_card_device(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_image_interface(mconfig, *this)
{
}

void mmc_card_device::device_start()
{
	save_item(NAME(m_saved_mount_id));
	save_item(NAME(m_card_state));
	save_item(NAME(m_rca));
	save_item(NAME(m_block_length));
	save_item(NAME(m_multiblock));
	save_item(NAME(m_next_address));
}

void mmc_card_device::device_reset()
{
	reset_protocol();
}

void mmc_card_device::reset_protocol()
{
	m_card_state = IDLE;
	m_rca = 1;
	m_block_length = SECTOR_BYTES;
	m_multiblock = false;
	m_next_address = 0;
}

bool mmc_card_device::revalidate_media()
{
	if (m_saved_mount_id == m_mount_id)
		return true;

	m_saved_mount_id = m_mount_id;
	reset_protocol();
	return false;
}

bool mmc_card_device::is_readable() const noexcept
{
	return true;
}

bool mmc_card_device::is_writeable() const noexcept
{
	return true;
}

bool mmc_card_device::is_creatable() const noexcept
{
	return false;
}

char const *mmc_card_device::image_type_name() const noexcept
{
	return "mmccard";
}

char const *mmc_card_device::image_brief_type_name() const noexcept
{
	return "mmc";
}

char const *mmc_card_device::file_extensions() const noexcept
{
	return "mmc,img,bin";
}

std::pair<std::error_condition, std::string> mmc_card_device::call_load()
{
	constexpr u64 MAX_IMAGE_BYTES = 0x40000000;

	auto const register_field = [](u8 *bytes, u32 high, u32 low, u32 value)
	{
		for (u32 bit = low; bit <= high; ++bit)
			if (BIT(value, bit - low))
				bytes[(127 - bit) / 8] |= 1U << (bit & 7);
	};

	auto const register_crc = [](u8 const *bytes) -> u8
	{
		u8 crc = 0;
		for (u32 i = 0; i < 15; ++i)
			for (int bit = 7; bit >= 0; --bit)
				crc = ((crc << 1) ^ ((BIT(crc, 6) != BIT(bytes[i], bit)) ? 9 : 0)) & 0x7f;
		return (crc << 1) | 1;
	};

	u64 const size = length();
	if (size < 4 * SECTOR_BYTES || size > MAX_IMAGE_BYTES + SECTOR_BYTES || size % SECTOR_BYTES)
		return { image_error::INVALIDLENGTH, "MMC image must contain whole 512-byte sectors (2 KiB to 1 GiB)" };

	u8 footer[SECTOR_BYTES];
	if (fseek(size - SECTOR_BYTES, SEEK_SET) || fread(footer, sizeof(footer)) != sizeof(footer))
		return { image_error::INVALIDIMAGE, "Could not read the last MMC sector" };

	u64 payload = size;
	u32 ocr = 0x00ff8000;
	u8 cid[16] = { 0, 0, 0, 'M', 'A', 'M', 'E', 'M', 'C', 0x10, 0, 0, 0, 1, 0xa8, 0 };
	u8 csd[16]{};
	bool const has_footer = !std::memcmp(footer, "NGAGEMMC", 8);
	if (has_footer)
	{
		if (footer[8] != 1)
			return { image_error::UNSUPPORTED, "Unsupported NGAGEMMC footer version" };
		payload -= SECTOR_BYTES;
		std::copy_n(&footer[16], 16, cid);
		std::copy_n(&footer[32], 16, csd);
		ocr = get_u32be(&footer[48]);
		if ((csd[0] >> 6) != 2 || (csd[5] & 15) != 9 || (ocr & 0x60000000))
			return { image_error::UNSUPPORTED, "Expected a byte-addressed MMC with 512-byte sectors" };
	}
	else
	{
		if (payload > MAX_IMAGE_BYTES)
			return { image_error::INVALIDLENGTH, "Raw MMC image exceeds 1 GiB" };
		u32 const sectors = payload / SECTOR_BYTES;
		u32 mult = 0;
		while (sectors > (4096U << (mult + 2)))
			++mult;
		u32 const sectors_per_unit = 1U << (mult + 2);
		if (sectors % sectors_per_unit)
			return { image_error::INVALIDLENGTH, "Raw MMC image size cannot be represented by a byte-addressed card" };
		u32 const count = sectors / sectors_per_unit;
		register_field(csd, 127, 126, 2);
		register_field(csd, 125, 122, 3);
		register_field(csd, 119, 112, 0x0f);
		register_field(csd, 103, 96, 0x2a);
		register_field(csd, 95, 84, 0x015);
		register_field(csd, 83, 80, 9);
		register_field(csd, 79, 79, 1);
		register_field(csd, 77, 77, 1);
		register_field(csd, 73, 62, count - 1);
		register_field(csd, 61, 50, 0xdb6);
		register_field(csd, 49, 47, mult);
		register_field(csd, 41, 37, 15);
		register_field(csd, 28, 26, 2);
		register_field(csd, 25, 22, 9);
	}

	u32 const c_size = ((csd[6] & 3) << 10) | (csd[7] << 2) | (csd[8] >> 6);
	u32 const mult = ((csd[9] & 3) << 1) | (csd[10] >> 7);
	u64 const capacity = u64(c_size + 1) << (mult + 11);
	if (payload > capacity)
		return { image_error::INVALIDLENGTH, "MMC payload is larger than the footer's CSD capacity" };

	std::random_device random;
	u64 const high = u64(random()) << 32;
	m_saved_mount_id = m_mount_id = (high | random()) | 1;
	m_payload_size = payload;
	m_card_capacity = capacity;
	m_ocr = ocr & 0x7fffffff;
	m_write_protected = is_readonly() || (csd[14] & 0x30) || payload < capacity;
	if (m_write_protected)
		csd[14] |= 0x10;
	cid[15] = register_crc(cid);
	csd[15] = register_crc(csd);
	std::copy_n(cid, 16, m_cid);
	std::copy_n(csd, 16, m_csd);
	return { };
}

void mmc_card_device::call_unload()
{
	m_saved_mount_id = m_mount_id = 0;
	m_payload_size = 0;
	reset_protocol();
	media_unloaded();
}

bool mmc_card_device::in_range(u64 offset, u32 length) const
{
	u64 const limit = std::min(m_card_capacity, m_payload_size);
	return offset <= limit && length <= limit - offset;
}

mmc_card_device::command_reply mmc_card_device::card_command(u8 command, u32 argument)
{
	if (!mounted())
		return { reply_type::no_media };

	u32 const r1 = (u32(m_card_state) << 9) | 0x100;
	bool const addressed = u16(argument >> 16) == m_rca;
	switch (command)
	{
	case 0:
		m_card_state = IDLE;
		m_rca = 1;
		m_block_length = SECTOR_BYTES;
		m_multiblock = false;
		return { };
	case 1:
		if (m_card_state == IDLE)
		{
			if (argument)
				m_card_state = READY;
			return { reply_type::short_response, m_ocr | 0x80000000 };
		}
		break;
	case 2:
		if (m_card_state == READY)
		{
			m_card_state = IDENT;
			return { reply_type::long_response, 0, m_cid };
		}
		break;
	case 3:
		if (m_card_state == IDENT)
		{
			m_rca = argument >> 16;
			m_card_state = STANDBY;
			return { reply_type::short_response, r1 };
		}
		break;
	case 7:
		if (addressed && (m_card_state == STANDBY || m_card_state == TRANSFER))
		{
			m_card_state = TRANSFER;
			return { reply_type::short_response, r1 };
		}
		if (m_card_state >= TRANSFER)
			m_card_state = STANDBY;
		return { reply_type::timeout };
	case 9:
	case 10:
		if (addressed && m_card_state == STANDBY)
			return { reply_type::long_response, 0, command == 9 ? m_csd : m_cid };
		break;
	case 12:
		if (m_card_state == DATA || m_card_state == RECEIVE || m_card_state == TRANSFER)
		{
			m_card_state = TRANSFER;
			m_multiblock = false;
			return { reply_type::short_response, r1 };
		}
		break;
	case 13:
		if (addressed && m_card_state >= STANDBY && m_card_state != INACTIVE)
			return { reply_type::short_response, r1 };
		return { reply_type::timeout };
	case 15:
		if (addressed)
			m_card_state = INACTIVE;
		return { };
	case 16:
		if (m_card_state == TRANSFER)
		{
			if (argument && argument <= SECTOR_BYTES && (BIT(m_csd[6], 7) || argument == SECTOR_BYTES))
			{
				m_block_length = argument;
				return { reply_type::short_response, r1 };
			}
			return { reply_type::short_response, r1 | BLOCK_LEN_ERROR };
		}
		break;
	case 17: case 18: case 24: case 25:
	{
		bool const write = command >= 24;
		u32 const card_class = write ? 4 : 2;
		u32 const classes = (m_csd[4] << 4) | (m_csd[5] >> 4);
		if (m_card_state != TRANSFER || !BIT(classes, card_class))
			break;

		u32 error = 0;
		if (write && m_write_protected)
			error = WRITE_PROTECTED;
		else if (write && m_block_length != SECTOR_BYTES)
			error = BLOCK_LEN_ERROR;
		else if ((write || !BIT(m_csd[6], 5)) && argument % m_block_length)
			error = ADDRESS_ERROR;
		else if (!in_range(argument, m_block_length))
			error = OUT_OF_RANGE;
		if (error)
			return { reply_type::short_response, r1 | error, nullptr, transfer_type::error };

		m_card_state = write ? RECEIVE : DATA;
		m_next_address = argument;
		m_multiblock = command == 18 || command == 25;
		return { reply_type::short_response, r1, nullptr, write ? transfer_type::write : transfer_type::read };
	}
	default:
		break;
	}
	return { reply_type::illegal, r1 | ILLEGAL_COMMAND };
}

mmc_card_device::data_result mmc_card_device::begin_transfer(bool write, u8 *buffer, u32 length)
{
	if (m_card_state != (write ? RECEIVE : DATA) || !is_open() || !in_range(m_next_address, length)
			|| (!m_multiblock && length > m_block_length) || (write && length % SECTOR_BYTES))
		return data_result::timeout;

	if (!write && (fseek(m_next_address, SEEK_SET) || fread(buffer, length) != length))
		return data_result::io_error;

	return data_result::ok;
}

bool mmc_card_device::finish_transfer(bool write, u8 const *buffer, u32 length)
{
	bool const success = !write || (mounted() && is_open() && m_saved_mount_id == m_mount_id
			&& !m_write_protected && in_range(m_next_address, length)
			&& !fseek(m_next_address, SEEK_SET) && fwrite(buffer, length) == length);
	m_next_address += length;
	if (!m_multiblock && (m_card_state == RECEIVE || m_card_state == DATA))
		m_card_state = TRANSFER;
	return success;
}
