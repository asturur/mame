// license:GPL-2.0+
// copyright-holders:Andrea Bogazzi, the NGage_MiSTer authors
/*
    N-Gage QD LCD, based on ngage2 model/hw/ngage_lcd.c (GPL-2.0-or-later).
    The UPP transfer phases carry DCS commands, parameters and packed pixels.
    The modeled 0x29 ID selects the QD driver's module 0x10 path.
*/

#include "emu.h"
#include "ngage_qd_lcd.h"

#include <algorithm>

DEFINE_DEVICE_TYPE(NGAGE_QD_LCD, ngage_qd_lcd_device, "ngage_qd_lcd", "N-Gage QD LCD")

ngage_qd_lcd_device::ngage_qd_lcd_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, NGAGE_QD_LCD, tag, owner, clock)
	, m_gram{}
	, m_response{}
	, m_parameters{}
	, m_lut{}
	, m_pixel{}
{
}

void ngage_qd_lcd_device::device_start()
{
	save_item(NAME(m_gram));
	save_item(NAME(m_control));
	save_item(NAME(m_aux));
	save_item(NAME(m_last_data));
	save_item(NAME(m_phase_left));
	save_item(NAME(m_command));
	save_item(NAME(m_colmod));
	save_item(NAME(m_madctl));
	save_item(NAME(m_sleep));
	save_item(NAME(m_display_on));
	save_item(NAME(m_partial));
	save_item(NAME(m_idle));
	save_item(NAME(m_tearing));
	save_item(NAME(m_partial_start));
	save_item(NAME(m_partial_end));
	save_item(NAME(m_col_start));
	save_item(NAME(m_col_end));
	save_item(NAME(m_row_start));
	save_item(NAME(m_row_end));
	save_item(NAME(m_col));
	save_item(NAME(m_row));
	save_item(NAME(m_response));
	save_item(NAME(m_read_size));
	save_item(NAME(m_read_pos));
	save_item(NAME(m_parameters));
	save_item(NAME(m_parameter_count));
	save_item(NAME(m_lut));
	save_item(NAME(m_lut_valid));
	save_item(NAME(m_pixel));
	save_item(NAME(m_pixel_size));
}

void ngage_qd_lcd_device::device_reset()
{
	m_control = 0;
	m_aux = 0;
	m_last_data = 0;
	m_phase_left = 0;
	reset_panel();
}

void ngage_qd_lcd_device::reset_panel()
{
	m_command = 0;
	m_colmod = 6;
	m_madctl = 0;
	m_sleep = true;
	m_display_on = false;
	m_partial = false;
	m_idle = false;
	m_tearing = false;
	m_partial_start = 0;
	m_partial_end = HEIGHT - 1;
	m_col_start = 0;
	m_col_end = WIDTH - 1;
	m_row_start = 0;
	m_row_end = HEIGHT - 1;
	m_col = 0;
	m_row = 0;
	m_read_size = 0;
	m_read_pos = 0;
	m_parameter_count = 0;
	m_lut_valid = false;
	m_pixel_size = 0;
}

u32 ngage_qd_lcd_device::read(offs_t offset)
{
	switch (offset)
	{
	case 0: return m_control | 0x40000000;
	case 1: return m_aux | 4;
	case 2:
		if ((m_control & 0x0f000000) != 0x07000000)
			return m_last_data;
		if (m_read_pos < m_read_size)
		{
			u8 const result = m_response[m_read_pos];
			if (!machine().side_effects_disabled())
				++m_read_pos;
			return result;
		}
		break;
	}
	return 0;
}

void ngage_qd_lcd_device::write(offs_t offset, u32 data)
{
	switch (offset)
	{
	case 0:
		m_control = data;
		m_phase_left = (data & 0xffff) + 1;
		break;
	case 1:
		m_aux = data;
		break;
	case 2:
		data_w(data);
		break;
	}
}

bool ngage_qd_lcd_device::dma_ready(u64 words) const
{
	u32 const packing = (m_control >> 18) & 3;
	u32 const bytes_per_word = packing == 1 ? 3 : 4;
	return writing_pixels() && (packing == 1 || packing == 3)
		&& m_phase_left && words <= (m_phase_left + bytes_per_word - 1) / bytes_per_word;
}

void ngage_qd_lcd_device::data_w(u32 data)
{
	m_last_data = data;
	u32 const packing = (m_control >> 18) & 3;
	if (packing == 2)
	{
		command_w(u8(data));
		return;
	}
	if (packing == 0)
		return;

	u32 const bytes = packing == 1 ? 3 : 4;
	if (packing == 1)
	{
		u32 const lo = data & 0x0fff;
		u32 const hi = (data >> 16) & 0x0fff;
		// Parameter phases serialize low byte first. The pixel phase
		// sends each RGB444 pixel most significant nibble first.
		data = BIT(m_control, 31)
			? (lo >> 4) | ((lo & 15) << 12) | ((hi >> 8) << 8) | ((hi & 0xff) << 16)
			: lo | (hi << 12);
	}
	for (u32 i = 0; i < bytes && m_phase_left; ++i, --m_phase_left)
	{
		if (writing_pixels())
			pixel_byte_w(u8(data));
		else
			parameter_w(u8(data));
		data >>= 8;
	}
}

void ngage_qd_lcd_device::command_w(u8 data)
{
	m_command = data;
	m_parameter_count = 0;
	m_read_size = m_read_pos = 0;
	if (data != 0x3c)
		m_pixel_size = 0;

	switch (data)
	{
	case 0x01: reset_panel(); break;
	case 0x04:
		m_response[0] = 0;
		m_response[1] = 0x29;
		m_response[2] = m_response[3] = 0;
		m_read_size = 4;
		break;
	case 0x09:
		m_response[0] = 0;
		m_response[1] = 0x80 | (m_madctl & 0xf4);
		m_response[2] = ((m_colmod & 7) << 4) | (m_idle ? 8 : 0) | (m_sleep ? 0 : 2) | 1;
		m_response[3] = (m_display_on ? 4 : 0) | (m_tearing ? 2 : 0);
		m_response[4] = 0;
		m_read_size = 5;
		break;
	case 0x10: m_sleep = true; break;
	case 0x11: m_sleep = false; break;
	case 0x12: m_partial = true; break;
	case 0x13: m_partial = false; break;
	case 0x28: m_display_on = false; break;
	case 0x29: m_display_on = true; break;
	case 0x2c:
		m_col = m_col_start;
		m_row = m_row_start;
		break;
	case 0x34: m_tearing = false; break;
	case 0x35: m_tearing = true; break;
	case 0x38: m_idle = false; break;
	case 0x39: m_idle = true; break;
	}
}

void ngage_qd_lcd_device::parameter_w(u8 data)
{
	if (m_parameter_count == std::size(m_parameters))
		return;
	m_parameters[m_parameter_count++] = data;
	u16 const first = (u16(m_parameters[0]) << 8) | m_parameters[1];
	u16 const last = (u16(m_parameters[2]) << 8) | m_parameters[3];
	switch (m_command)
	{
	case 0x2a:
		if (m_parameter_count == 4)
		{
			m_col_start = first;
			m_col_end = last;
		}
		break;
	case 0x2b:
		if (m_parameter_count == 4)
		{
			m_row_start = first;
			m_row_end = last;
		}
		break;
	case 0x2d:
		if (m_parameter_count == std::size(m_lut))
		{
			std::copy(std::begin(m_parameters), std::end(m_parameters), std::begin(m_lut));
			m_lut_valid = true;
		}
		break;
	case 0x30:
		if (m_parameter_count == 4)
		{
			m_partial_start = first;
			m_partial_end = last;
		}
		break;
	case 0x36:
		if (m_parameter_count == 1)
			m_madctl = data;
		break;
	case 0x3a:
		if (m_parameter_count == 1)
			m_colmod = data;
		break;
	}
}

rgb_t ngage_qd_lcd_device::rgb444(u8 r, u8 g, u8 b) const
{
	if (m_lut_valid)
		return rgb_t(pal5bit(m_lut[r]), pal6bit(m_lut[16 + g]), pal5bit(m_lut[32 + b]));
	return rgb_t(pal4bit(r), pal4bit(g), pal4bit(b));
}

void ngage_qd_lcd_device::pixel_byte_w(u8 data)
{
	m_pixel[m_pixel_size++] = data;
	if (m_colmod == 5)
	{
		if (m_pixel_size == 2)
		{
			u16 const value = (u16(m_pixel[0]) << 8) | m_pixel[1];
			pixel_w(rgb_t(pal5bit(value >> 11), pal6bit(value >> 5), pal5bit(value)));
			m_pixel_size = 0;
		}
	}
	else if (m_pixel_size == 3)
	{
		pixel_w(rgb444(m_pixel[0] >> 4, m_pixel[0] & 15, m_pixel[1] >> 4));
		pixel_w(rgb444(m_pixel[1] & 15, m_pixel[2] >> 4, m_pixel[2] & 15));
		m_pixel_size = 0;
	}
}

void ngage_qd_lcd_device::pixel_w(rgb_t color)
{
	u32 x = m_col;
	u32 y = m_row;
	if (BIT(m_madctl, 5))
		std::swap(x, y);
	if (BIT(m_madctl, 6))
		x = WIDTH - 1 - x;
	if (BIT(m_madctl, 7))
		y = HEIGHT - 1 - y;
	if (x < WIDTH && y < HEIGHT)
		m_gram[y][x] = color;

	if (m_col >= m_col_end)
	{
		m_col = m_col_start;
		m_row = m_row >= m_row_end ? m_row_start : m_row + 1;
	}
	else
		++m_col;
}

void ngage_qd_lcd_device::draw(bitmap_rgb32 &bitmap, rectangle const &cliprect) const
{
	if (m_sleep || !m_display_on)
	{
		bitmap.fill(rgb_t::black(), cliprect);
		return;
	}

	// The panel is mounted rotated 180 degrees in the QD reference.
	for (int y = cliprect.top(); y <= cliprect.bottom(); ++y)
	{
		u32 const gram_row = HEIGHT - 1 - y;
		u32 const addressed_row = BIT(m_madctl, 7) ? y : gram_row;
		for (int x = cliprect.left(); x <= cliprect.right(); ++x)
		{
			u32 pixel = rgb_t::black();
			if (!m_partial || (addressed_row >= m_partial_start && addressed_row <= m_partial_end))
			{
				pixel = m_gram[gram_row][WIDTH - 1 - x];
				if (m_idle)
					pixel = rgb_t(BIT(pixel, 23) ? 255 : 0, BIT(pixel, 15) ? 255 : 0, BIT(pixel, 7) ? 255 : 0);
			}
			bitmap.pix(y, x) = pixel;
		}
	}
}
