SOURCES := utils.c disasm.c emulator.c riscv.c pipeline.c cache.c
HEADERS := types.h utils.h riscv.h pipeline.h stage_helpers.h cache.h config.h
PWD := $(shell pwd)
CUNIT := -L $(PWD)/CUnit-install/lib -I $(PWD)/CUnit-install/include -llibcunit
CFLAGS := -g  -Wall

all: riscv

riscv: $(SOURCES) $(HEADERS)
	gcc $(CFLAGS) -o $@ $(SOURCES)

test-utils: test_utils.c utils.c $(HEADERS)
	gcc $(CFLAGS) -DTESTING -o test-utils test_utils.c utils.c $(CUNIT)
	./test-utils
	rm -f test-utils

clean:
	rm -f riscv
	rm -f *.o *~
	rm -f test-utils
	rm -f code/ms*/out/*.solution code/ms*/out/*/*.solution
	rm -f code/ms*/out/*.trace code/ms*/out/*/*.trace

deepclean: clean
	rm -rf CUnit-install
