// license:GPL-2.0+
// copyright-holders:Andrea Bogazzi, the NGage_MiSTer authors
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
	static constexpr unsigned WIDTH = 176;
	static constexpr unsigned HEIGHT = 208;
	void reset_panel();
	void command_w(u8 data);
	void data_w(u32 data);
	void parameter_w(u8 data);
	void pixel_byte_w(u8 data);
	void pixel_w(rgb_t color);
	rgb_t rgb444(u8 r, u8 g, u8 b) const;
	bool writing_pixels() const { return m_command == 0x2c || m_command == 0x3c; }

	u32 m_gram[HEIGHT][WIDTH]{};
	u32 m_control = 0;
	u32 m_aux = 0;
	u32 m_last_data = 0;
	u32 m_phase_left = 0;

	u8 m_command = 0;
	u8 m_colmod = 6;
	u8 m_madctl = 0;
	bool m_sleep = true;
	bool m_display_on = false;
	bool m_partial = false;
	bool m_idle = false;
	bool m_tearing = false;
	u16 m_partial_start = 0;
	u16 m_partial_end = HEIGHT - 1;
	u16 m_col_start = 0;
	u16 m_col_end = WIDTH - 1;
	u16 m_row_start = 0;
	u16 m_row_end = HEIGHT - 1;
	u16 m_col = 0;
	u16 m_row = 0;

	u8 m_response[5]{};
	u8 m_read_size = 0;
	u8 m_read_pos = 0;
	u8 m_parameters[48]{};
	u8 m_parameter_count = 0;
	u8 m_lut[48]{};
	bool m_lut_valid = false;
	u8 m_pixel[3]{};
	u8 m_pixel_size = 0;
};

DECLARE_DEVICE_TYPE(NGAGE_QD_LCD, ngage_qd_lcd_device)

#endif // MAME_NOKIA_NGAGE_QD_LCD_H
