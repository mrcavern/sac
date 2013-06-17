CFLAGS += -O2 -g -Wall
sac: sac.c
clean:
	rm -f sac

update:
	cp ~/sketchbook/sac/sac.ino sac.c
	rpl "ARDUINO_MODE 1" "ARDUINO_MODE 0" sac.c
