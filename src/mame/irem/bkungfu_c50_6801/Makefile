CRASM ?= crasm
PYTHON ?= python3

all: bkungfu_c50_6801.bin

bkungfu_c50_6801.srec: firmware.asm
	$(CRASM) -l -o $@ $<

bkungfu_c50_6801.bin: bkungfu_c50_6801.srec srec_to_bin.py
	$(PYTHON) srec_to_bin.py $< $@

clean:
	rm -f bkungfu_c50_6801.bin bkungfu_c50_6801.srec

.PHONY: all clean
