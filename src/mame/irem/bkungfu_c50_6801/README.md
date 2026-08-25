# Experimental Beyond Kung-Fu C50 replacement firmware

This is a bring-up target for MAME's `M6801U4` core.  It does not identify the
undumped Irem C50 as a Motorola part.  The surrogate has 192 bytes of internal
RAM and 4 KiB of internal ROM, which makes it a useful constraint for firmware
that can later be translated to an FPGA MCU.

The external address map is deliberately a development contract, not a claim
about the original PCB:

| 6801 address | Function |
|---|---|
| `1000-8fff` | Raw encrypted `km-z.4h` data ROM |
| `9000-97ff` | Z80 `c800-cfff` shared mailbox |
| `a000-afff` | 4 KiB page-mirrored tilemap write bus |
| `b000-efff` | Full 16 KiB level tilemap write bus |
| `f000-ffff` | 6801U4 internal firmware ROM |

The cipher is intended to run per ROM read.  There is no 32 KiB plaintext
buffer: the recovered first-stage transform is address-local, and commands
only consume the selected pointer chains and streams.

The MC6801U4 is configured from its normal 4 MHz input clock and given a MAME
clock scale of 6.  This is explicitly a throughput model for the unknown C50,
not a claim that a physical MC6801U4 ran at that rate.  It makes command `02`
complete in about 14 video frames, matching the observed loading cadence:
the level finishes, the Z80 reveals and walks the player, then its independently
timed stage-name/`READY` sequence begins.

Current firmware status:

- `fe`: startup plus all observed ready/status bytes;
- `08`: full tilemap clear through the page VRAM bus;
- `0a`: static HUD ROM script;
- `0d`/`14`: persistent-state ROM text scripts;
- `0f`: encrypted object records and streams, including both dragons, flame
  frames, and later-stage animation objects;
- `01`/`02`: zero-based half-level selection (`00`-`0f`) and direct
  encrypted-stream expansion into the complete 56-column playfield, without a
  decoded level buffer or a synthetic background prefill; unloaded columns
  retain command `08`'s black clear until their encrypted streams are consumed;
- `0c`: shared-RAM high-score text scripts;
- `10`: two packed-BCD credits/continue digits;
- all eight `c810-c82f` HUD slots: packed-BCD scores and timer, floor dots,
  lives, player energy and boss energy;
- `05`: parameterless drawing-phase boundary, acknowledged with no independent
  ROM or VRAM operation observed at any Z80 call site.

The on-demand cipher, payload transform, object renderer, and ROM text grammar
and level renderer are now 6801 code.  The known RAM text and dynamic HUD
operations are also firmware-owned.  All observed main commands and HUD slots
therefore have explicit firmware behavior; the acknowledge-only nature of
command `05` remains documented as a protocol inference.

Build with the CRASM copy already present in the local JTCORES checkout:

```sh
make CRASM=/Users/andrea/develop/jtcores/modules/jt680x/crasm/src/crasm
```

The resulting `bkungfu_c50_6801.bin` is a development ROM.  Put it in the
`bkungfu` set archive (or a loose `bkungfu` ROM directory) on the first MAME
ROM path.  A firmware-only `bkungfu.zip` can precede the untouched original
archive in a semicolon-separated `-rompath`; MAME successfully merges the two.
Generated `.bin` and `.srec` files are not source-controlled.
