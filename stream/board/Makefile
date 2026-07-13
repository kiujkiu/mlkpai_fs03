# pov_rxd - board-side PVS1 stream receiver (v2: 三缓冲/双线程/DELTA/26pps)
#
# Targets:
#   make / make arm   cross-compile static ARM binaries `pov_rxd` + `bench_s0`
#                     (ship in git, board needs no toolchain). Links
#                     deps/arm/libz.a. NEON on (Zynq A9 has it).
#   make sim          x86 dev build: pov_rxd_sim (-DSIM_NO_DEVMEM) + test_local
#   make test         build sim + run the loopback protocol test
#   make bench        bench_s0 (ARM static) + bench_s0_x86 (local sanity)
#   make ko           povmem.ko WC-mmap kernel module (needs board kernel
#                     headers: make -C povmem KDIR=... ARCH=arm
#                     CROSS_COMPILE=arm-linux-gnueabihf-)
#   make deps         (re)build deps/arm/{libz.a,zlib.h,zconf.h} from zlib
#                     1.3.1 source (downloads via curl; only needed if the
#                     committed prebuilt libz.a is deleted)
#   make clean

CROSS   ?= arm-linux-gnueabihf-
DEPS    := deps/arm
CFLAGS  := -O2 -Wall -Wextra -pthread
ARMTUNE := -march=armv7-a -mfpu=neon -mfloat-abi=hard
PROTO   := ../protocol.h

all: arm
arm: pov_rxd bench_s0

pov_rxd: pov_rxd.c $(PROTO) $(DEPS)/libz.a
	$(CROSS)gcc $(CFLAGS) $(ARMTUNE) -static -I$(DEPS) -o $@ pov_rxd.c $(DEPS)/libz.a
	$(CROSS)strip $@
	file $@ || true

bench_s0: bench_s0.c $(PROTO) $(DEPS)/libz.a
	$(CROSS)gcc $(CFLAGS) $(ARMTUNE) -static -I$(DEPS) -o $@ bench_s0.c $(DEPS)/libz.a
	$(CROSS)strip $@

sim: pov_rxd_sim test_local

pov_rxd_sim: pov_rxd.c $(PROTO)
	gcc $(CFLAGS) -DSIM_NO_DEVMEM -o $@ pov_rxd.c -lz

test_local: test_local.c $(PROTO)
	gcc $(CFLAGS) -o $@ test_local.c -lz

test: sim
	./test_local

bench: bench_s0 bench_s0_x86

bench_s0_x86: bench_s0.c $(PROTO)
	gcc $(CFLAGS) -o $@ bench_s0.c -lz

ko:
	$(MAKE) -C povmem

# Rebuild the prebuilt static zlib for ARM (needs network + cross gcc).
ZLIB_VER := 1.3.1
ZLIB_URL := https://github.com/madler/zlib/archive/refs/tags/v$(ZLIB_VER).tar.gz
deps:
	mkdir -p $(DEPS) build
	curl -sL -o build/zlib.tar.gz $(ZLIB_URL)
	tar -C build -xzf build/zlib.tar.gz
	cd build/zlib-$(ZLIB_VER) && CC=$(CROSS)gcc CFLAGS=-O2 ./configure --static && $(MAKE) libz.a
	cp build/zlib-$(ZLIB_VER)/libz.a build/zlib-$(ZLIB_VER)/zlib.h \
	   build/zlib-$(ZLIB_VER)/zconf.h $(DEPS)/
	rm -rf build

clean:
	rm -f pov_rxd pov_rxd_sim test_local bench_s0 bench_s0_x86
	rm -rf build
	-$(MAKE) -C povmem clean

.PHONY: all arm sim test bench ko deps clean
