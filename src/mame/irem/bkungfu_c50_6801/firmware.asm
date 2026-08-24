;;; license:GPL-2.0+
;;; copyright-holders:Andrea Bogazzi
;;;
;;; Experimental replacement firmware for the undumped Beyond Kung-Fu C50.
;;; This targets an MC6801U4 as a development surrogate; it is not a claim
;;; that the original Irem custom device contained a Motorola CPU.

	cpu 6801
	output scode

mailbox        = $9000
page_vram      = $a000

scratch_tile   = $40
scratch_attr   = $41
scratch_romptr = $42
scratch_cipher = $44
scratch_sum    = $45
scratch_value  = $46
object_width   = $47
object_column  = $48
object_rowskip = $49
object_dest    = $4a
object_stream  = $4c
script_persist = $4e

	* = $f000

reset
	sei
	lds  #$00ff

;;; The Z80 RAM test leaves the mailbox clear, then writes command $fe.
wait_start
	ldaa mailbox
	cmpa #$fe
	bne  wait_start
	jsr  startup

main_loop
	jsr  service_command
	jsr  service_hud_slots
	bra  main_loop

startup
	ldaa #$fe
	staa mailbox+$10
	staa mailbox+$14
	staa mailbox+$18
	staa mailbox+$1c
	staa mailbox+$20
	staa mailbox+$24
	staa mailbox+$28
	staa mailbox+$2c
	staa mailbox+$102
	staa mailbox+$106
	staa mailbox+$118
	staa mailbox+$11c
	staa mailbox
	rts

service_command
	ldaa mailbox
	cmpa #$fe
	beq  command_done

	cmpa #$08
	beq  command_clear

	;;; Explicitly recognized commands.  Their implementations will replace
	;;; these acknowledgements one at a time as the HLE algorithms are ported.
	cmpa #$01
	beq  command_ack
	cmpa #$02
	beq  command_ack
	cmpa #$05
	beq  command_ack
	cmpa #$0a
	bne  command_check_0c
	jmp  command_hud
command_check_0c
	cmpa #$0c
	beq  command_ack
	cmpa #$0d
	bne  command_check_0f
	jmp  command_text
command_check_0f
	cmpa #$0f
	bne  command_check_10
	jmp  command_object
command_check_10
	cmpa #$10
	beq  command_ack
	cmpa #$14
	bne  command_done
	jmp  command_text

	;;; Unknown commands remain busy so an incomplete protocol cannot be hidden.
command_done
	rts

command_ack
	ldaa #$fe
	staa mailbox
	rts

;;; Command $08 is the first real drawing primitive.  C801 is the attribute
;;; and C802 is the tile byte.  The MAME-side bus expands this 4 KiB page over
;;; the four currently mirrored tilemap pages, just as the HLE model does.
command_clear
	ldaa mailbox+2
	staa scratch_tile
	ldaa mailbox+1
	staa scratch_attr
	ldx  #page_vram

clear_loop
	ldaa scratch_tile
	staa 0,x
	inx
	ldaa scratch_attr
	staa 0,x
	inx
	cpx  #$b000
	bne  clear_loop
	bra  command_ack

;;; Commands $0d/$14 select a two-byte pointer in the plaintext ROM directory.
;;; Command $0a uses the fixed pointer at ROM $0140 for the base HUD.  All use
;;; the same stream grammar: $01 attr, $02 position-low/high, literals, $00 end.
command_hud
	clr  script_persist
	clr  scratch_attr
	clr  object_dest
	clr  object_dest+1
	ldx  #$1140              ; physical bus address for ROM offset $0140
	bra  start_rom_script

command_text
	ldaa #$01
	staa script_persist
	ldaa mailbox+2
	staa object_dest+1
	ldaa mailbox+3
	anda #$0f
	oraa #$a0
	staa object_dest
	ldaa mailbox+4
	staa scratch_attr
	ldaa mailbox+1
	staa scratch_value
	asla
	tab
	ldx  #$1000
	abx
	ldaa scratch_value
	bita #$80                ; retain the ninth bit of 2 * table index
	beq  start_rom_script
	stx  scratch_romptr
	inc  scratch_romptr
	ldx  scratch_romptr

start_rom_script
	jsr  decrypt_data
	staa object_stream+1
	inx
	jsr  decrypt_data
	adda #$10                ; ROM offset -> physical bus address
	staa object_stream

rom_script_loop
	ldx  object_stream
	jsr  decrypt_data
	staa scratch_value
	inx
	stx  object_stream
	ldaa scratch_value
	beq  rom_script_complete
	cmpa #$01
	beq  rom_script_attr
	cmpa #$02
	beq  rom_script_position

	ldx  object_dest
	staa 0,x
	inx
	ldaa scratch_attr
	staa 0,x

	;;; Advance two bytes within the current 64-tile row without carrying into
	;;; the row bits, matching `(position + 2) & $7f` in the HLE model.
	ldaa object_dest+1
	staa scratch_cipher
	adda #$02
	anda #$7f
	staa scratch_value
	ldaa scratch_cipher
	anda #$80
	oraa scratch_value
	staa object_dest+1
	ldaa script_persist
	beq  rom_script_loop
	ldaa object_dest+1
	staa mailbox+2
	bra  rom_script_loop

rom_script_attr
	ldx  object_stream
	jsr  decrypt_data
	staa scratch_attr
	ldab script_persist
	beq  rom_script_attr_advance
	staa mailbox+4
rom_script_attr_advance
	inx
	stx  object_stream
	bra  rom_script_loop

rom_script_position
	ldx  object_stream
	jsr  decrypt_data
	staa object_dest+1
	ldab script_persist
	beq  rom_script_position_high
	staa mailbox+2
rom_script_position_high
	inx
	jsr  decrypt_data
	staa scratch_value
	ldab script_persist
	beq  rom_script_position_map
	staa mailbox+3
rom_script_position_map
	ldaa scratch_value
	anda #$0f
	oraa #$a0
	staa object_dest
	inx
	stx  object_stream
	bra  rom_script_loop

rom_script_complete
	jmp  command_ack

;;; Command $0f object records are selected by C801.  The plaintext directory
;;; at ROM $0100 points to five-byte records: width, destination, stream.
;;; Stream bytes are first-stage decrypted on access; literal tile bytes and
;;; attributes following $5f then receive the common payload transform.
command_object
	ldaa mailbox+1
	cmpa #$80
	bcc  object_id_ge_80
	jmp  command_ack
object_id_ge_80
	cmpa #$91
	bcs  object_id_valid
	jmp  command_ack
object_id_valid
	suba #$80
	asla
	tab
	ldx  #$1100              ; physical bus address for ROM offset $0100
	abx

	jsr  decrypt_data
	staa scratch_value       ; record pointer low
	inx
	jsr  decrypt_data
	adda #$10                ; ROM offset -> physical bus address
	staa scratch_romptr
	ldaa scratch_value
	staa scratch_romptr+1
	ldx  scratch_romptr

	jsr  decrypt_data
	staa object_width
	inx
	jsr  decrypt_data
	staa object_dest+1
	inx
	jsr  decrypt_data
	anda #$0f
	oraa #$a0                ; page-VRAM bus base
	staa object_dest
	inx
	jsr  decrypt_data
	staa object_stream+1
	inx
	jsr  decrypt_data
	adda #$10                ; ROM offset -> physical bus address
	staa object_stream

	ldaa object_width
	asla
	staa scratch_value
	ldaa #$80
	suba scratch_value
	staa object_rowskip
	clr  object_column
	clr  scratch_attr

object_loop
	ldx  object_stream
	jsr  decrypt_data
	staa scratch_value
	inx
	stx  object_stream
	ldaa scratch_value
	cmpa #$5f
	beq  object_set_attr
	cmpa #$5e
	beq  object_complete
	cmpa #$60
	beq  object_complete

	ldx  object_stream
	dex                         ; address of the literal just consumed
	jsr  decode_payload
	ldx  object_dest
	staa 0,x
	inx
	ldaa scratch_attr
	staa 0,x
	inx
	stx  object_dest

	inc  object_column
	ldaa object_column
	cmpa object_width
	bne  object_loop
	clr  object_column
	ldab object_rowskip
	ldx  object_dest
	abx
	stx  object_dest
	bra  object_loop

object_set_attr
	ldx  object_stream
	jsr  decode_payload
	staa scratch_attr
	inx
	stx  object_stream
	bra  object_loop

object_complete
	jmp  command_ack

;;; Each HUD trigger is the first byte of a four-byte mailbox slot.  Dynamic
;;; drawing is not ported yet, but ownership and completion already reside in
;;; the firmware rather than in the Z80 driver.
service_hud_slots
	ldx  #mailbox+$10

hud_slot_loop
	ldaa 0,x
	beq  hud_slot_next
	cmpa #$fe
	beq  hud_slot_next
	ldaa #$fe
	staa 0,x

hud_slot_next
	inx
	inx
	inx
	inx
	cpx  #mailbox+$30
	bne  hud_slot_loop
	rts

;;; Decode one byte from the raw external data ROM.  X is a physical surrogate
;;; bus address ($1000 + original ROM offset), is preserved, and A returns the
;;; first-stage plaintext byte.  There is deliberately no plaintext buffer.
decrypt_data
	stx  scratch_romptr
	ldaa 0,x
	staa scratch_cipher
	cpx  #$253d              ; physical address for ROM offset $153d
	bcs  decrypt_restore

	ldaa scratch_romptr      ; high byte of physical address
	suba #$10                ; convert to original ROM high byte
	adda scratch_romptr+1
	staa scratch_sum
	tab
	ldx  #cipher_key
	abx
	ldaa 0,x
	ldab scratch_sum
	andb #$01
	beq  decrypt_even
	suba scratch_cipher
	bra  decrypt_restore

decrypt_even
	eora scratch_cipher

decrypt_restore
	ldx  scratch_romptr
	rts

;;; Apply the second, address-dependent payload transform used by object and
;;; level tile/attribute literals.  X is preserved and A returns the bus byte.
decode_payload
	jsr  decrypt_data
	staa scratch_value
	ldab scratch_sum
	andb #$01
	beq  payload_even
	ldaa #$60
	suba scratch_value
	rts

payload_even
	ldaa scratch_value
	bita #$20
	beq  payload_base_60
	ldaa #$a0
	bra  payload_subtract

payload_base_60
	ldaa #$60

payload_subtract
	suba scratch_value
	ldab scratch_value
	andb #$01
	bne  payload_done
	suba #$02

payload_done
	rts

default_isr
	rti

cipher_key
	db $ae,$f3,$5c,$5d,$aa,$f7,$58,$59,$a6,$fb,$54,$55,$a2,$ff,$50,$51
	db $be,$e3,$4c,$4d,$ba,$e7,$48,$49,$b6,$eb,$44,$45,$b2,$ef,$40,$41
	db $8e,$13,$7c,$3d,$8a,$17,$78,$39,$86,$1b,$74,$35,$82,$1f,$70,$31
	db $9e,$03,$6c,$2d,$9a,$07,$68,$29,$96,$0b,$64,$25,$92,$0f,$60,$21
	db $ee,$33,$1c,$1d,$ea,$37,$18,$19,$e6,$3b,$14,$15,$e2,$3f,$10,$11
	db $fe,$23,$0c,$0d,$fa,$27,$08,$09,$f6,$2b,$04,$05,$f2,$2f,$00,$01
	db $ce,$53,$3c,$fd,$ca,$57,$38,$f9,$c6,$5b,$34,$f5,$c2,$5f,$30,$f1
	db $de,$43,$2c,$ed,$da,$47,$28,$e9,$d6,$4b,$24,$e5,$d2,$4f,$20,$e1
	db $2e,$73,$dc,$dd,$2a,$77,$d8,$d9,$26,$7b,$d4,$d5,$22,$7f,$d0,$d1
	db $3e,$63,$cc,$cd,$3a,$67,$c8,$c9,$36,$6b,$c4,$c5,$32,$6f,$c0,$c1
	db $0e,$93,$fc,$bd,$0a,$97,$f8,$b9,$06,$9b,$f4,$b5,$02,$9f,$f0,$b1
	db $1e,$83,$ec,$ad,$1a,$87,$e8,$a9,$16,$8b,$e4,$a5,$12,$8f,$e0,$a1
	db $6e,$b3,$9c,$9d,$6a,$b7,$98,$99,$66,$bb,$94,$95,$62,$bf,$90,$91
	db $7e,$a3,$8c,$8d,$7a,$a7,$88,$89,$76,$ab,$84,$85,$72,$af,$80,$81
	db $4e,$d3,$bc,$7d,$4a,$d7,$b8,$79,$46,$db,$b4,$75,$42,$df,$b0,$71
	db $5e,$c3,$ac,$6d,$5a,$c7,$a8,$69,$56,$cb,$a4,$65,$52,$cf,$a0,$61

	* = $fff0
	dw default_isr            ; SCI
	dw default_isr            ; timer overflow
	dw default_isr            ; output compare
	dw default_isr            ; input capture
	dw default_isr            ; external IRQ
	dw default_isr            ; SWI
	dw default_isr            ; NMI
	dw reset                  ; reset

	code
