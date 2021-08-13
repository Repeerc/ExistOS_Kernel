
SCRIPT_DIR	= ../scripts

ELF = ../../../../../nuttx
BIN = ../../../../../nuttx.bin

ELF2ROM = ./elftosb.exe
SBLOADER =./sb_loader.exe


firmware.sb:
	cp $(BIN) .
	$(ELF2ROM) -z -c $(SCRIPT_DIR)/build_fw.bd -o firmware.sb nuttx.bin

clean:
	rm *.bin
	rm firmware.sb
	
flash: firmware.sb
	$(SBLOADER) -f firmware.sb

all: firmware.sb
	
	