// license:BSD-3-Clause
// copyright-holders:Andrea Bogazzi, Jacob Jewell, the NGage_MiSTer authors
/*
 * CBUS Bluetooth controller from the N-Gage QD study's Bluetooth 1.1 model.
 * See the NGage_MiSTer Bluetooth notes for the firmware traces.
 * Portions copyright (c) 2026 the NGage_MiSTer authors.
 *
 * Registers (16 bits each; the number is the CBUS register, the name the
 * driver's where it prints one):
 *
 *   0x00  data. Read: the next two bytes of the packet the module holds
 *         for the phone, first byte in bits 15:8. Write: the next two bytes
 *         of the phone's packet, the same way round.
 *   0x01  length in bytes of the packet the module holds (read only).
 *   0x06  read: status. Bits 4-8 are events, cleared by the read; the
 *         interrupt line pulses when one is set.
 *           bit 2  awake (always, here)
 *           bit 4  a packet for the phone is ready (registers 0x01, 0x00)
 *           bit 5  the module wants to sleep (never, here)
 *           bit 6  register 0x08 has an answer
 *           bit 7  the module has reset (register 0x09 says why)
 *           bit 8  the module woke up (never, here)
 *         write: bit 0 resets the module, bit 1 wakes it.
 *   0x08  "LPRF_MCU_Reg0". Write: 0x8000 | n asks for room for a packet of
 *         n bytes, 0xC000 | n says its n bytes have been written. Read: the
 *         answer, n in bits 13:0 with bits 15:14 = 00 for the room and 01
 *         for the packet.
 *   0x09  reset reason; bit 3 = the phone asked for it.
 *   0x0A  write 0x8000 after a reset: the module answers through register
 *         0x08 with the size of its buffer.
 */

#include "emu.h"
#include "ngage_bt.h"

#include <algorithm>
#include <iterator>

/*
 * The state is what the core's copy keeps, register for register, so that
 * the two can be compared after every access: the module does not buffer
 * packets. Of the phone's packet it keeps the bytes a command is decoded
 * from (tx[0], tx[2..8]; tx[] is whole here only for the log) and the
 * words of a new name; its own packet is made a word at a time, as the
 * phone reads it, from what the last command left (rep_*).
 */
struct ngage_bt_device::chip_state
{
	static constexpr u8 BT_REG_DATA = 0x00;
	static constexpr u8 BT_REG_RXLEN = 0x01;
	static constexpr u8 BT_REG_STATUS = 0x06;
	static constexpr u8 BT_REG_REG0 = 0x08;
	static constexpr u8 BT_REG_REASON = 0x09;
	static constexpr u8 BT_REG_START = 0x0a;

	static constexpr u16 BT_ST_AWAKE = 0x0004;
	static constexpr u16 BT_ST_RX = 0x0010;
	static constexpr u16 BT_ST_ACK = 0x0040;
	static constexpr u16 BT_ST_RESET = 0x0080;

	static constexpr u16 BT_REASON_HOST = 0x0008;
	static constexpr u32 BT_BUF_SIZE = 520; // the driver's largest write
	static constexpr u32 BT_NAME_LEN = 248;
	static constexpr u32 BT_NPAR = 16;

	/* HCI status codes used here. */
	static constexpr u8 BT_OK = 0x00;
	static constexpr u8 BT_UNKNOWN_CMD = 0x01;
	static constexpr u8 BT_DISALLOWED = 0x0c;
	static constexpr u8 BT_BAD_PARAMS = 0x12;

	/* What the module holds for the phone (rx_kind) and where a command's
	 * return values come from (rep_src). */
	enum : u8 { BT_RX_NONE, BT_RX_REPLY, BT_RX_INQ };
	enum : u8 { BT_SRC_NONE, BT_SRC_FIX, BT_SRC_PAR, BT_SRC_NAME };

	/*
	 * The values a command reads back before the phone has written them: the
	 * defaults of the Bluetooth 1.1 specification. {read OCF, bytes, default},
	 * all in OGF 3; the write command is the next OCF.
	 */
	struct bt_parameter { u8 ocf, n, def[4]; };
	static constexpr bt_parameter BT_PARAMETERS[BT_NPAR] = {
		{ 0x09, 1, { 0x00 } },                      /* PIN type: variable */
		{ 0x15, 2, { 0xa0, 0x1f } },                /* connection accept timeout: 5.06 s */
		{ 0x17, 2, { 0x00, 0x20 } },                /* page timeout: 5.12 s */
		{ 0x19, 1, { 0x00 } },                      /* scan enable: none */
		{ 0x1b, 4, { 0x00, 0x08, 0x12, 0x00 } },    /* page scan interval 1.28 s, window 11.25 ms */
		{ 0x1d, 4, { 0x00, 0x08, 0x12, 0x00 } },    /* inquiry scan, the same */
		{ 0x1f, 1, { 0x00 } },                      /* authentication: off */
		{ 0x21, 1, { 0x00 } },                      /* encryption: off */
		{ 0x23, 3, { 0x00, 0x00, 0x00 } },          /* class of device */
		{ 0x25, 2, { 0x60, 0x00 } },                /* voice setting */
		{ 0x29, 1, { 0x01 } },                      /* broadcast retransmissions */
		{ 0x2b, 1, { 0x00 } },                      /* hold mode activity */
		{ 0x2e, 1, { 0x00 } },                      /* SCO flow control: off */
		{ 0x39, 4, { 0x01, 0x33, 0x8b, 0x9e } },    /* one inquiry access code, the general one */
		{ 0x3b, 1, { 0x00 } },                      /* page scan period mode P0 */
		{ 0x3d, 1, { 0x00 } },                      /* page scan mode: mandatory */
	};
	static constexpr u32 BT_PAR_IAC = 13;

	// Commands that complete at once with fixed values.
	struct bt_reply { u16 op; u8 n, ret[8]; };
	static constexpr bt_reply BT_FIXED_REPLIES[] = {
		{ 0x0c0d, 4, { 0x00, 0x00, 0x00, 0x00 } },  /* Read_Stored_Link_Key: room for none, none read */
		{ 0x0c11, 1, { 0x00 } },                    /* Write_Stored_Link_Key: none written */
		{ 0x0c12, 2, { 0x00, 0x00 } },              /* Delete_Stored_Link_Key: none deleted */
		{ 0x0c38, 1, { 0x01 } },                    /* Read_Number_Of_Supported_IAC */
		/* Read_Local_Version_Information: HCI 1.1, revision 0, LMP 1.1,
		 * manufacturer 1 (Nokia Mobile Phones), subversion 0 */
		{ 0x1001, 8, { 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00 } },
		/* Read_Local_Supported_Features: all of 1.1's */
		{ 0x1003, 8, { 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00 } },
		/* Read_Buffer_Size: ACL 339 bytes x 8, SCO 64 bytes x 8 */
		{ 0x1005, 7, { 0x53, 0x01, 0x40, 0x08, 0x00, 0x08, 0x00 } },
		{ 0x1007, 1, { 0x00 } },                    /* Read_Country_Code */
		{ 0x1009, 6, { 0x00, 0x00, 0x00, 0x57, 0x60, 0x00 } },  /* Read_BD_ADDR: 00:60:57:00:00:00 */
		/* nothing to return: Set_Event_Mask, Set_Event_Filter, Create_New_Unit_Key,
		 * Set_Host_Controller_To_Host_Flow_Control, Host_Buffer_Size */
		{ 0x0c01, 0, { 0 } }, { 0x0c05, 0, { 0 } }, { 0x0c0b, 0, { 0 } },
		{ 0x0c31, 0, { 0 } }, { 0x0c33, 0, { 0 } },
	};

	void event(u16 bit);
	void begin_reset();
	void finish_reset();
	u8 rx_byte(u32 i) const;
	void publish_reply();
	void finish_inquiry();
	u16 read(u8 reg);
	void write(u8 reg, u16 value);
	void command(u32 op, u32 p, u32 plen);
	void packet(u32 len);


	bool in_reset;              /* waiting for the reset timer */
	u16 events;                /* status bits 4-8, cleared when read */
	u16 reg0, reason;
	/* the phone's packet */
	u8 tx[BT_BUF_SIZE];
	u16 tx_idx;                /* bytes written since room was asked for */
	u16 name_w[128];           /* words 2-126 of the last Change_Local_Name, as written */
	/* the controller */
	u32 par[BT_NPAR];           /* HCI parameter values, first byte lowest */
	u16 par_set;               /* bit e: par[e] was written since the reset */
	u16 name_len;
	bool inq_run, inq_done;         /* an inquiry runs; it has ended and its event waits */
	/* the answer to the last command */
	bool rep_pend;                  /* it waits for the phone to take what is held */
	bool rep_cs;                    /* Command Status (else Command Complete) */
	u8 rep_status, rep_src, rep_idx;
	u16 rep_op, rep_n;
	/* the packet held for the phone */
	u8 rx_kind;
	u16 rx_len, rx_idx;
	/* for the owner (cleared by it) */
	bool irq;                       /* an event was raised: pulse the line */
	bool reset_req;             /* start the reset timer */
	u8 inq_start;              /* nonzero: start the inquiry timer, this many 1.28 s */
	bool inq_stop;                  /* stop the inquiry timer */
};

void ngage_bt_device::chip_state::event(u16 bit)
{
	events |= bit;
	irq = true;
}

/* The phone asked for a reset: everything is forgotten at once and the
 * module is deaf (reads 0, ignores writes) until the owner's reset time
 * has passed. */
void ngage_bt_device::chip_state::begin_reset()
{
	*this = {};
	in_reset = true;
	reset_req = true;
	inq_stop = true;
}

/* The module has come out of that reset. */
void ngage_bt_device::chip_state::finish_reset()
{
	in_reset = false;
	reason = BT_REASON_HOST;
	event(BT_ST_RESET);
}

/* Byte i of the packet held for the phone. An event is 04 00, its code,
 * its parameter length, its parameters. */
u8 ngage_bt_device::chip_state::rx_byte(u32 i) const
{
	if (rx_kind == BT_RX_INQ)
	{
		/* Inquiry Complete: one parameter, the status. hci.dll (0x5046bce8)
		 * takes no other length: with two bytes (a status and a count of
		 * responses) it hands the stack "Invalid HCI Command Parameters". */
		static constexpr u8 INQUIRY_COMPLETE[5] = { 0x04, 0x00, 0x01, 0x01, 0x00 };
		return i < 5 ? INQUIRY_COMPLETE[i] : 0;
	}
	if (rep_cs)
	{
		/* Command Status: status, one more command may be sent, opcode */
		const u8 cs[8] = { 0x04, 0x00, 0x0f, 0x04, rep_status, 0x01,
								u8(rep_op), u8(rep_op >> 8) };
		return i < 8 ? cs[i] : 0;
	}
	if (i < 8)
	{
		/* Command Complete: one more command may be sent, opcode, status */
		const u8 cc[8] = { 0x04, 0x00, 0x0e, u8(4 + rep_n), 0x01,
								u8(rep_op), u8(rep_op >> 8), rep_status };
		return cc[i];
	}
	u32 const k = i - 8;
	u32 const e = rep_idx;
	switch (rep_src)
	{
	case BT_SRC_FIX:
		return BT_FIXED_REPLIES[e].ret[k & 7];
	case BT_SRC_PAR:
		return BIT(par_set, e) ? u8(par[e] >> (8 * (k & 3))) : BT_PARAMETERS[e].def[k & 3];
	case BT_SRC_NAME:
		/* name byte k was byte k + 5 of the packet that set it */
		if (k < name_len)
		{
			u16 w = name_w[(k + 5) >> 1 & 127];
			return (k + 5) & 1 ? u8(w) : u8(w >> 8);
		}
		return 0;
	}
	return 0;
}

/* Put up the next packet for the phone, if it has taken the last one: a
 * command's answer first, then the end of an inquiry. */
void ngage_bt_device::chip_state::publish_reply()
{
	if (rx_kind != BT_RX_NONE)
	{
		return;
	}
	if (rep_pend)
	{
		rep_pend = false;
		rx_kind = BT_RX_REPLY;
		rx_len = rep_cs ? 8 : 8 + rep_n;
	}
	else if (inq_done)
	{
		inq_done = false;
		rx_kind = BT_RX_INQ;
		rx_len = 5;
	}
	else
	{
		return;
	}
	rx_idx = 0;
	event(BT_ST_RX);
}

/* The inquiry's time is up (the owner's timer). */
void ngage_bt_device::chip_state::finish_inquiry()
{
	if (inq_run)
	{
		inq_run = false;
		inq_done = true;
		publish_reply();
	}
}

u16 ngage_bt_device::chip_state::read(u8 reg)
{
	u16 v = 0;

	if (in_reset)
	{
		return 0;
	}
	switch (reg)
	{
	case BT_REG_DATA:
		if (rx_kind != BT_RX_NONE)
		{
			v = rx_byte(rx_idx) << 8;
			if (rx_idx + 1 < rx_len)
			{
				v |= rx_byte(rx_idx + 1);
			}
			rx_idx += 2;
			if (rx_idx >= rx_len)
			{
				rx_kind = BT_RX_NONE;    /* the phone has all of it */
				publish_reply();
			}
		}
		break;
	case BT_REG_RXLEN:
		v = rx_kind != BT_RX_NONE ? rx_len : 0;
		break;
	case BT_REG_STATUS:
		v = events | BT_ST_AWAKE;
		events = 0;
		break;
	case BT_REG_REG0:
		v = reg0;
		break;
	case BT_REG_REASON:
		v = reason;
		break;
	}
	return v;
}

void ngage_bt_device::chip_state::write(u8 reg, u16 v)
{
	u32 n = v & 0x3fff;

	if (reg == BT_REG_STATUS && (v & 1))
	{
		begin_reset();
		return;
	}
	if (in_reset)
	{
		return;
	}
	switch (reg)
	{
	case BT_REG_DATA:
	{
		u32 const w = tx_idx >> 1;
		if (tx_idx < BT_BUF_SIZE)
		{
			tx[tx_idx++] = v >> 8;
			tx[tx_idx++] = v & 0xff;
			if (w >= 2 && w <= 126 && tx[2] == 0x13 && tx[3] == 0x0c)
			{
				name_w[w] = v;           /* a Change_Local_Name's name, as it comes */
			}
		}
		break;
	}
	case BT_REG_START:
		if (v & 0x8000)
		{
			reg0 = BT_BUF_SIZE;
			event(BT_ST_ACK);
		}
		break;
	case BT_REG_REG0:
		if ((v & 0xc000) == 0x8000)           /* room for n bytes? */
		{
			tx_idx = 0;
			reg0 = n;
			event(BT_ST_ACK);
		}
		else if ((v & 0xc000) == 0xc000)    /* n bytes written */
		{
			u32 const got = tx_idx;
			tx_idx = 0;
			reg0 = 0x4000 | n;
			event(BT_ST_ACK);
			if (n > BT_BUF_SIZE)
			{
				n = BT_BUF_SIZE;
			}
			if (got >= n)                     /* all of it was written */
			{
				packet(n);
			}
		}
		break;
	}
}

/*
 * HCI: a Bluetooth 1.1 controller with nobody in range.
 *
 * The phone's packet is tx[0..len): 01 00, then the command (opcode, low
 * byte first; the parameter length; the parameters). One command is
 * answered at a time, as HCI's own flow control has it: a command that
 * comes before the last one's answer was read takes that answer's place.
 */
void ngage_bt_device::chip_state::command(u32 op, u32 p, u32 plen)
{
	u32 const ogf = op >> 10;
	u32 const ocf = op & 0x3ff;
	bool rep = true, cs = false;
	u8 status = BT_OK;
	u8 src = BT_SRC_NONE;
	u32 idx = 0;
	u32 n = 0;
	bool known = false;

	for (u32 i = 0; i < std::size(BT_FIXED_REPLIES); i++)
	{
		if (BT_FIXED_REPLIES[i].op == op)
		{
			known = true;
			n = BT_FIXED_REPLIES[i].n;
			src = n ? BT_SRC_FIX : BT_SRC_NONE;
			idx = n ? i : 0;
		}
	}
	for (u32 e = 0; e < BT_NPAR && ogf == 3 && !known; e++)
	{
		if (ocf == BT_PARAMETERS[e].ocf)                     /* Read_... */
		{
			known = true;
			src = BT_SRC_PAR;
			idx = e;
			n = BT_PARAMETERS[e].n;
		}
		else if (ocf == BT_PARAMETERS[e].ocf + 1u)         /* Write_... */
		{
			known = true;
			if (plen < BT_PARAMETERS[e].n || (e == BT_PAR_IAC && (p & 0xff) != 1))
			{
				status = BT_BAD_PARAMS;
			}
			else
			{
				par[e] = p;
				par_set |= 1u << e;
			}
		}
	}
	if (!known)
	{
		switch (op)
		{
		case 0x0401:                            /* Inquiry: LAP, length, responses */
			cs = true;
			if (plen < 5 || (p >> 24) < 1 || (p >> 24) > 0x30)
			{
				status = BT_BAD_PARAMS;
			}
			else if (inq_run)
			{
				status = BT_DISALLOWED;
			}
			else
			{
				inq_run = true;
				inq_start = p >> 24;
			}
			break;
		case 0x0402:                            /* Inquiry_Cancel */
			if (inq_run)
			{
				inq_run = false;
				inq_stop = true;
			}
			else
			{
				status = BT_DISALLOWED;
			}
			break;
		case 0x0c03:                            /* Reset */
			inq_run = inq_done = false;
			inq_stop = true;
			par_set = 0;
			name_len = 0;
			break;
		case 0x0c13:                            /* Change_Local_Name */
			name_len = plen < BT_NAME_LEN ? plen : BT_NAME_LEN;
			break;
		case 0x0c14:                            /* Read_Local_Name */
			src = BT_SRC_NAME;
			n = BT_NAME_LEN;
			break;
		case 0x0c35:                            /* Host_Number_Of_Completed_Packets: no event */
			rep = false;
			break;
		default:
			/* With no link and nobody to page, the link commands cannot
			 * run; anything else is not a command of this controller. */
			cs = true;
			status = ogf == 1 || ogf == 2 ? BT_DISALLOWED : BT_UNKNOWN_CMD;
			break;
		}
	}
	if (rep)
	{
		rep_pend = true;
		rep_cs = cs;
		rep_status = status;
		rep_op = op;
		rep_src = src;
		rep_idx = idx;
		rep_n = n;
		if (rx_kind == BT_RX_REPLY)
		{
			rx_kind = BT_RX_NONE;            /* the answer nobody read */
		}
	}
}

void ngage_bt_device::chip_state::packet(u32 len)
{
	if (len < 5 || tx[0] != 0x01)
	{
		return;                                 /* not a command: there is no link for data */
	}
	u32 const plen = std::min<u32>(tx[4], len - 5);
	command(tx[2] | (u32(tx[3]) << 8),
				   tx[5] | (u32(tx[6]) << 8) | (u32(tx[7]) << 16) | (u32(tx[8]) << 24), plen);
	publish_reply();
}

DEFINE_DEVICE_TYPE(NGAGE_BT, ngage_bt_device, "ngage_bt", "N-Gage LRB-2 Bluetooth")

ngage_bt_device::ngage_bt_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, NGAGE_BT, tag, owner, clock)
	, m_chip(std::make_unique<chip_state>())
	, m_irq_cb(*this)
{
}

ngage_bt_device::~ngage_bt_device() = default;

void ngage_bt_device::device_start()
{
	m_reset_timer = timer_alloc(FUNC(ngage_bt_device::reset_done), this);
	m_inquiry_timer = timer_alloc(FUNC(ngage_bt_device::inquiry_done), this);

	save_item(NAME(m_chip->in_reset));
	save_item(NAME(m_chip->events));
	save_item(NAME(m_chip->reg0));
	save_item(NAME(m_chip->reason));
	save_item(NAME(m_chip->tx));
	save_item(NAME(m_chip->tx_idx));
	save_item(NAME(m_chip->name_w));
	save_item(NAME(m_chip->par));
	save_item(NAME(m_chip->par_set));
	save_item(NAME(m_chip->name_len));
	save_item(NAME(m_chip->inq_run));
	save_item(NAME(m_chip->inq_done));
	save_item(NAME(m_chip->rep_pend));
	save_item(NAME(m_chip->rep_cs));
	save_item(NAME(m_chip->rep_status));
	save_item(NAME(m_chip->rep_src));
	save_item(NAME(m_chip->rep_idx));
	save_item(NAME(m_chip->rep_op));
	save_item(NAME(m_chip->rep_n));
	save_item(NAME(m_chip->rx_kind));
	save_item(NAME(m_chip->rx_len));
	save_item(NAME(m_chip->rx_idx));
}

void ngage_bt_device::device_reset()
{
	*m_chip = {};
	m_reset_timer->adjust(attotime::never);
	m_inquiry_timer->adjust(attotime::never);
}

u16 ngage_bt_device::read(u8 reg)
{
	u16 const data = m_chip->read(reg);
	update();
	return data;
}

void ngage_bt_device::write(u8 reg, u16 data)
{
	m_chip->write(reg, data);
	update();
}

void ngage_bt_device::update()
{
	chip_state &chip = *m_chip;
	// The QD model uses a provisional 10 ms controller reset delay.
	// An inquiry length unit is 1.28 s in Bluetooth 1.1.
	if (chip.inq_stop)
		m_inquiry_timer->adjust(attotime::never);
	if (chip.inq_start)
		m_inquiry_timer->adjust(attotime::from_msec(1280 * chip.inq_start));
	if (chip.reset_req)
		m_reset_timer->adjust(attotime::from_msec(10));

	chip.inq_stop = chip.reset_req = false;
	chip.inq_start = 0;
	if (chip.irq)
	{
		chip.irq = false;
		m_irq_cb(1);
		m_irq_cb(0);
	}
}

TIMER_CALLBACK_MEMBER(ngage_bt_device::reset_done)
{
	m_chip->finish_reset();
	update();
}

TIMER_CALLBACK_MEMBER(ngage_bt_device::inquiry_done)
{
	m_chip->finish_inquiry();
	update();
}
