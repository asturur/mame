// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi

#include "emu.h"
#include "ngage_mmc.h"

#include "multibyte.h"

#include <algorithm>

DEFINE_DEVICE_TYPE(NGAGE_MMC, ngage_mmc_device, "ngage_mmc", "N-Gage MMC slot")

ngage_mmc_device::ngage_mmc_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: mmc_card_device(mconfig, NGAGE_MMC, tag, owner, clock)
	, m_event_cb(*this)
	, m_media_change_cb(*this)
	, m_command_timer(nullptr)
	, m_tail_timer(nullptr)
	, m_hot_swap(false)
	, m_response{}
	, m_fifo{}
{
}

bool ngage_mmc_device::is_reset_on_load() const noexcept
{
	return !m_hot_swap;
}

void ngage_mmc_device::device_start()
{
	mmc_card_device::device_start();
	m_command_timer = timer_alloc(FUNC(ngage_mmc_device::command_complete), this);
	m_tail_timer = timer_alloc(FUNC(ngage_mmc_device::command_tail), this);
	save_item(NAME(m_reg));
	save_item(NAME(m_status));
	save_item(NAME(m_type));
	save_item(NAME(m_command));
	save_item(NAME(m_argument));
	save_item(NAME(m_response));
	save_item(NAME(m_response_words));
	save_item(NAME(m_response_pos));
	save_item(NAME(m_fifo_direction));
	save_item(NAME(m_fifo));
	save_item(NAME(m_fifo_length));
	save_item(NAME(m_fifo_pos));
}

void ngage_mmc_device::device_reset()
{
	mmc_card_device::device_reset();
	std::fill(std::begin(m_reg), std::end(m_reg), 0);
	reset_host();
}

void ngage_mmc_device::device_post_load()
{
	if (!revalidate_media())
	{
		abort_card_transfer();
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(ngage_mmc_device::notify_media_change), this));
	}
}

void ngage_mmc_device::reset_host()
{
	m_status = 0;
	m_type = m_command = m_argument = 0;
	m_response_words = m_response_pos = 0;
	m_fifo_direction = FIFO_IDLE;
	m_fifo_length = m_fifo_pos = 0;
	m_command_timer->enable(false);
	m_tail_timer->enable(false);
}

void ngage_mmc_device::abort_card_transfer()
{
	u32 const status = ((m_status & CMD_BUSY) && m_type ? RESP_TIMEOUT : 0)
		| (m_status & DATA_BUSY ? DATA_TIMEOUT : 0);
	reset_host();
	m_status = status;
}

TIMER_CALLBACK_MEMBER(ngage_mmc_device::notify_media_change)
{
	m_media_change_cb(1);
}

void ngage_mmc_device::media_unloaded()
{
	bool const running = machine().phase() == machine_phase::RUNNING;
	if (m_hot_swap && running)
		abort_card_transfer();
	else if (m_command_timer)
	{
		std::fill(std::begin(m_reg), std::end(m_reg), 0);
		reset_host();
	}
	if (running)
		m_media_change_cb(1);
}

u32 ngage_mmc_device::irq_sources() const
{
	return ((m_status & DATA_BUSY) ? 0 : (1U << 21))
		| ((m_status & CMD_BUSY) ? 0 : (1U << 23))
		| ((m_status & RESP_READY) ? (1U << 24) : 0);
}

bool ngage_mmc_device::dma_ready(bool write) const
{
	return m_fifo_direction == (write ? FIFO_TX : FIFO_RX) && m_fifo_pos < m_fifo_length;
}

u32 ngage_mmc_device::read(offs_t offset, u32 mem_mask)
{
	switch (offset)
	{
	case 0x04 / 4: return m_reg[offset] & ~1U;
	case 0x18 / 4: return m_status;
	case 0x20 / 4: return response_r();
	case 0x24 / 4: return fifo_r(mem_mask);
	default: return m_reg[offset];
	}
}

void ngage_mmc_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset == 0x24 / 4)
	{
		fifo_w(data, mem_mask);
		return;
	}
	COMBINE_DATA(&m_reg[offset]);
	if (offset == 0x04 / 4 && BIT(data & mem_mask, 0))
		start(m_reg[offset]);
}

void ngage_mmc_device::start(u32 control)
{
	m_type = (control >> 2) & 15;
	m_command = m_reg[0x1c / 4] & 63;
	m_argument = m_reg[0x20 / 4];
	m_response_words = m_response_pos = 0;
	m_fifo_direction = FIFO_IDLE;
	m_fifo_pos = m_fifo_length = 0;
	m_status = 0;
	m_command_timer->enable(false);
	m_tail_timer->enable(false);

	bool const continuation = m_type == 6 || m_type == 9;
	bool const data_first = m_type == 7 || m_type == 10;
	if (!continuation)
		m_status |= CMD_BUSY;
	if (m_type >= 5 && m_type <= 10)
		m_status |= DATA_BUSY;
	if (continuation || data_first)
		begin_data(m_type <= 7);
	else
		m_command_timer->adjust(clocks_to_attotime(48 + (m_type == 3 ? 136 : 48)));
	m_event_cb(1);
}

TIMER_CALLBACK_MEMBER(ngage_mmc_device::command_complete)
{
	complete_card_command();
	if (m_response_words != 4)
		m_tail_timer->adjust(clocks_to_attotime(8));
	m_event_cb(1);
}

TIMER_CALLBACK_MEMBER(ngage_mmc_device::command_tail)
{
	m_status &= ~CMD_BUSY;
	m_event_cb(1);
}

void ngage_mmc_device::response(u32 value)
{
	if (m_type)
	{
		m_response[0] = value;
		m_response_words = 1;
		m_status |= RESP_READY;
	}
}

void ngage_mmc_device::response(u8 const *bytes)
{
	for (u32 word = 0; word < 4; ++word)
		m_response[word] = get_u32be(bytes + word * 4);
	m_response_words = 4;
	m_status |= RESP_READY;
}

u32 ngage_mmc_device::response_r()
{
	if (m_response_pos == m_response_words)
		return 0;
	u32 const data = m_response[m_response_pos];
	if (!machine().side_effects_disabled())
	{
		++m_response_pos;
		// NEM-4 reads the last R2 word on CmdIdle, the first three on CmdRdy.
		if (m_response_words == 4 && m_response_pos == 3)
			m_status &= ~CMD_BUSY;
		if (m_response_pos == m_response_words)
			m_status &= ~RESP_READY;
		m_event_cb(1);
	}
	return data;
}

void ngage_mmc_device::complete_card_command()
{
	auto const reply = card_command(m_command, m_argument);
	switch (reply.type)
	{
	case reply_type::none:
		break;
	case reply_type::short_response:
		response(reply.value);
		break;
	case reply_type::long_response:
		response(reply.register_data);
		break;
	case reply_type::timeout:
		if (m_type)
			m_status |= RESP_TIMEOUT;
		break;
	case reply_type::no_media:
		if (m_type)
			m_status |= RESP_TIMEOUT;
		m_status &= ~DATA_BUSY;
		break;
	case reply_type::illegal:
		if (m_type == 3)
			m_status |= RESP_TIMEOUT;
		else
			response(reply.value);
		if (m_status & DATA_BUSY)
			m_status = (m_status & ~DATA_BUSY) | DATA_TIMEOUT;
		break;
	}

	switch (reply.transfer)
	{
	case transfer_type::none:
		break;
	case transfer_type::read:
	case transfer_type::write:
		if (m_status & DATA_BUSY)
			begin_data(reply.transfer == transfer_type::write);
		break;
	case transfer_type::error:
		m_status = (m_status & ~DATA_BUSY) | DATA_TIMEOUT;
		break;
	}
}

void ngage_mmc_device::begin_data(bool write)
{
	u32 const programmed = m_reg[0x08 / 4];
	u32 const size = programmed ? std::min<u32>(programmed, sizeof(m_fifo)) : block_length();
	switch (begin_transfer(write, m_fifo, size))
	{
	case data_result::timeout:
		m_status |= DATA_TIMEOUT;
		end_data();
		return;
	case data_result::io_error:
		m_status |= DATA_ERROR;
		end_data();
		return;
	case data_result::ok:
		break;
	}
	m_fifo_length = size;
	m_fifo_pos = 0;
	m_fifo_direction = write ? FIFO_TX : FIFO_RX;
}

void ngage_mmc_device::end_data()
{
	if (!finish_transfer(m_fifo_direction == FIFO_TX, m_fifo, m_fifo_length))
		m_status |= DATA_ERROR;
	m_fifo_direction = FIFO_IDLE;
	m_status &= ~DATA_BUSY;
	if (m_type == 7 || m_type == 10)
		m_command_timer->adjust(clocks_to_attotime(96));
	m_event_cb(1);
}

u32 ngage_mmc_device::fifo_r(u32 mask)
{
	if (m_fifo_direction != FIFO_RX)
		return 0;

	u32 data = 0;
	u32 idx = m_fifo_pos;
	for (u32 lane = 0; lane < 32 && idx < m_fifo_length; lane += 8)
		if (mask & (0xffU << lane))
			data |= u32(m_fifo[idx++]) << lane;
	if (!machine().side_effects_disabled())
	{
		m_fifo_pos = idx;
		if (m_fifo_pos == m_fifo_length)
			end_data();
	}
	return data;
}

void ngage_mmc_device::fifo_w(u32 data, u32 mask)
{
	if (m_fifo_direction != FIFO_TX)
		return;

	for (u32 lane = 0; lane < 32 && m_fifo_pos < m_fifo_length; lane += 8)
		if (mask & (0xffU << lane))
			m_fifo[m_fifo_pos++] = data >> lane;
	if (m_fifo_pos == m_fifo_length)
		end_data();
}
