// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi
#ifndef MAME_NOKIA_NGAGE_ADSP_H
#define MAME_NOKIA_NGAGE_ADSP_H

#pragma once

#include <span>

class ngage_adsp_device : public device_t, public device_memory_interface
{
public:
	static constexpr feature_type unemulated_features() { return feature::SOUND; }

	ngage_adsp_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	void power_w(bool enabled);
	void tx_w(u8 data);
	bool rx_pending() const { return m_output_count != 0; }
	u8 rx_r();

protected:
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;
	virtual space_config_vector memory_space_config() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr u32 BOOT_RAM_BYTES = 0x40000;
	static constexpr u32 D551_BASE = 0x400000;
	enum : u8 { WAIT_LINK, READY, WAIT_ACK };

	void program_map(address_map &map) ATTR_COLD;
	bool load_boot_ram();
	void reset_link();
	void handle_frame();
	void send_frame(u8 resource, std::span<u8 const> payload);
	void send_startup(bool reset_only);

	static constexpr u32 MAX_PAYLOAD = 247;
	static constexpr u32 MAX_FRAME = 10 + MAX_PAYLOAD + 1;

	address_space_config const m_program_config;
	required_region_ptr<u8> m_d551;
	required_shared_ptr<u8> m_boot_ram;

	bool m_enabled;
	bool m_boot_loaded;
	u8 m_state;
	u8 m_frame[MAX_FRAME];
	u16 m_frame_size;
	u8 m_output[64];
	u8 m_output_head;
	u8 m_output_count;
	u8 m_device_sequence;
	u8 m_request_sequence;
	u8 m_request[MAX_PAYLOAD];
	u8 m_request_size;
	u8 m_reply[12];
	u8 m_reply_size;
};

DECLARE_DEVICE_TYPE(NGAGE_ADSP, ngage_adsp_device)

#endif // MAME_NOKIA_NGAGE_ADSP_H
