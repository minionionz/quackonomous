#
# Makefile for vim users :)
#

PIO_ENV=

all:
	pio -f -c vim run ${PIO_ENV}

monitor:
	pio device monitor ${PIO_ENV} -b 115200

# run whatever other target like 'build', 'upload' or 'clean'
%:
	pio -f -c vim run ${PIO_ENV} --target $@
