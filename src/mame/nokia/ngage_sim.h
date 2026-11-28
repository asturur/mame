// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi
#ifndef MAME_NOKIA_NGAGE_SIM_H
#define MAME_NOKIA_NGAGE_SIM_H

#pragma once

class ngage_sim_device : public device_t
{
public:
	ngage_sim_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	void set_present(bool present) { m_present = present; }
	u16 status() const { return m_status; }
	void acknowledge(u16 bits);
	void start(u16 command);
	u8 read();
	void write(u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	struct test_file;
	static test_file const *find_file(u16 id);
	void reply_status(u16 status);
	void command(u8 length);

	enum : u8 { ATR, TX_COMPLETE, RX };

	bool m_present;
	bool m_started;
	u16 m_status;
	u8 m_phase;
	u8 m_position;
	u8 m_reply[25];
	u8 m_length;
	u16 m_directory;
	u16 m_selected;
	u16 m_file_id;
	u8 m_header[4];
	u8 m_header_count;
	u8 m_select_left;
	u8 m_terminal_left;
};

DECLARE_DEVICE_TYPE(NGAGE_SIM, ngage_sim_device)

#endif // MAME_NOKIA_NGAGE_SIM_H
