TARGETS = PRU0 PRU1

all:
	make -C PRU0 install
	make -C PRU1 install
	sudo rmmod -f pru_rproc
	sudo modprobe pru_rproc

