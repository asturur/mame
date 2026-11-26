// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi, Jacob Jewell, the NGage_MiSTer authors
#ifndef MAME_NOKIA_NGAGE_QD_LCD_H
#define MAME_NOKIA_NGAGE_QD_LCD_H

#pragma once

class ngage_qd_lcd_device : public device_t
{
public:
	ngage_qd_lcd_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	u32 read(offs_t offset);
	void write(offs_t offset, u32 data);
	bool dma_ready(u64 words) const;
	void draw(bitmap_rgb32 &bitmap, rectangle const &cliprect) const;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr u32 WIDTH = 176;
	static constexpr u32 HEIGHT = 208;
	void reset_panel();
	void command_w(u8 data);
	void data_w(u32 data);
	void parameter_w(u8 data);
	void pixel_byte_w(u8 data);
	void pixel_w(rgb_t color);
	rgb_t rgb444(u8 r, u8 g, u8 b) const;
	bool writing_pixels() const { return m_command == 0x2c || m_command == 0x3c; }

	u32 m_gram[HEIGHT][WIDTH];
	u32 m_control;
	u32 m_aux;
	u32 m_last_data;
	u32 m_phase_left;

	u8 m_command;
	u8 m_colmod;
	u8 m_madctl;
	bool m_sleep;
	bool m_display_on;
	bool m_partial;
	bool m_idle;
	bool m_tearing;
	u16 m_partial_start;
	u16 m_partial_end;
	u16 m_col_start;
	u16 m_col_end;
	u16 m_row_start;
	u16 m_row_end;
	u16 m_col;
	u16 m_row;

	u8 m_response[5];
	u8 m_read_size;
	u8 m_read_pos;
	u8 m_parameters[48];
	u8 m_parameter_count;
	u8 m_lut[48];
	bool m_lut_valid;
	u8 m_pixel[3];
	u8 m_pixel_size;
};

DECLARE_DEVICE_TYPE(NGAGE_QD_LCD, ngage_qd_lcd_device)

#endif // MAME_NOKIA_NGAGE_QD_LCD_H
