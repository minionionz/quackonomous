#
# Makefile for vim users :)
#

all:
	pio -f -c vim run

monitor:
	pio device monitor -b 115200

# run whatever other target like 'build', 'upload' or 'clean'
%:
	pio -f -c vim run --target $@
