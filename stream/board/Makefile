# pov_rxd - board-side PVS1 stream receiver
#
# Targets:
#   make / make arm   cross-compile static ARM binary `pov_rxd` (ships in git,
#                     board needs no toolchain). Links deps/arm/libz.a.
#   make sim          x86 dev build: pov_rxd_sim (-DSIM_NO_DEVMEM) + test_local
#   make test         build sim + run the loopback protocol test
#   make deps         (re)build deps/arm/{libz.a,zlib.h,zconf.h} from zlib
#                     1.3.1 source (downloads via curl; only needed if the
#                     committed prebuilt libz.a is deleted)
#   make clean

CROSS   ?= arm-linux-gnueabihf-
DEPS    := deps/arm
CFLAGS  := -O2 -Wall -Wextra -fno-strict-aliasing
PROTO   := ../protocol.h pvs_codec.h

all: arm
arm: pov_rxd

pov_rxd: pov_rxd.c $(PROTO) $(DEPS)/libz.a
	$(CROSS)gcc $(CFLAGS) -static -I$(DEPS) -o $@ pov_rxd.c $(DEPS)/libz.a
	$(CROSS)strip $@
	file $@ || true

sim: pov_rxd_sim test_local bench_delta

pov_rxd_sim: pov_rxd.c $(PROTO)
	gcc $(CFLAGS) -DSIM_NO_DEVMEM -o $@ pov_rxd.c -lz

test_local: test_local.c $(PROTO)
	gcc $(CFLAGS) -o $@ test_local.c -lz

bench_delta: bench_delta.c $(PROTO)
	gcc $(CFLAGS) -o $@ bench_delta.c -lz

test: sim
	./test_local

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
	rm -f pov_rxd pov_rxd_sim test_local bench_delta
	rm -rf build

.PHONY: all arm sim test deps clean
