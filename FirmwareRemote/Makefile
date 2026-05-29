#
# Makefile for vim users :)
#

PIO_ENV=supermini

all:
	pio -f -c vim run -e ${PIO_ENV}

monitor:
	pio device monitor -e ${PIO_ENV} -b 115200

# run whatever other target like 'build', 'upload' or 'clean'
%:
	pio -f -c vim run -e ${PIO_ENV} --target $@
