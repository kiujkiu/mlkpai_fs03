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
#   make deps-lz4     同上, 但重建 deps/arm/{liblz4.a,lz4.h,lz4hc.h} (lz4 1.10.0)
#   make clean
#
# 为什么 lz4 也走「交叉编译静态库 + committed 进仓库」而不是链板上的
# liblz4.so.1.8.3: 板子上没有开发包 (apt 装不到 lz4/liblz4-dev), 而且这里的
# ARM 产物是 **-static** 的 —— 静态可执行文件既不能 -l:liblz4.so.1, dlopen()
# 在静态 glibc 下也基本不可用。详见 pov_rxd.c 里 #include <lz4.h> 上方的注释。

CROSS   ?= arm-linux-gnueabihf-
DEPS    := deps/arm
CFLAGS  := -O2 -Wall -Wextra -pthread
ARMTUNE := -march=armv7-a -mfpu=neon -mfloat-abi=hard
PROTO   := ../protocol.h

all: arm
arm: pov_rxd bench_s0

pov_rxd: pov_rxd.c $(PROTO) $(DEPS)/libz.a $(DEPS)/liblz4.a
	$(CROSS)gcc $(CFLAGS) $(ARMTUNE) -static -I$(DEPS) -o $@ pov_rxd.c \
	    $(DEPS)/libz.a $(DEPS)/liblz4.a
	$(CROSS)strip $@
	file $@ || true

bench_s0: bench_s0.c $(PROTO) $(DEPS)/libz.a
	$(CROSS)gcc $(CFLAGS) $(ARMTUNE) -static -I$(DEPS) -o $@ bench_s0.c $(DEPS)/libz.a
	$(CROSS)strip $@

sim: pov_rxd_sim test_local

# x86 侧的 lz4: 头文件借 deps/arm/lz4.h (纯 C 头, 与架构无关), 库用
# `-l:liblz4.so.1` 直接点名 soname —— 这样**不需要 liblz4-dev**, 只要发行版
# 的运行时包 liblz4-1 在 (WSL/Debian 默认就有)。用 -llz4 的话会去找
# liblz4.so 这个只有 dev 包才装的符号链接。
LZ4_X86 := -l:liblz4.so.1

pov_rxd_sim: pov_rxd.c $(PROTO)
	gcc $(CFLAGS) -DSIM_NO_DEVMEM -I$(DEPS) -o $@ pov_rxd.c -lz $(LZ4_X86)

test_local: test_local.c $(PROTO)
	gcc $(CFLAGS) -I$(DEPS) -o $@ test_local.c -lz $(LZ4_X86)

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

# Rebuild the prebuilt static lz4 for ARM (needs network + cross gcc).
# 只有解码器 (lz4.o) 会被 pov_rxd 链进去; lz4hc.o 一并打包是为了以后想在板上
# 压缩/做 bench 时不用重新取源码。
LZ4_VER := 1.10.0
LZ4_URL := https://github.com/lz4/lz4/archive/refs/tags/v$(LZ4_VER).tar.gz
deps-lz4:
	mkdir -p $(DEPS) build
	curl -sL -o build/lz4.tar.gz $(LZ4_URL)
	tar -C build -xzf build/lz4.tar.gz
	cd build/lz4-$(LZ4_VER)/lib && \
	  $(CROSS)gcc -O3 -Wall $(ARMTUNE) -c lz4.c lz4hc.c && \
	  $(CROSS)ar rcs liblz4.a lz4.o lz4hc.o
	cp build/lz4-$(LZ4_VER)/lib/liblz4.a build/lz4-$(LZ4_VER)/lib/lz4.h \
	   build/lz4-$(LZ4_VER)/lib/lz4hc.h $(DEPS)/
	rm -rf build

clean:
	rm -f pov_rxd pov_rxd_sim test_local bench_s0 bench_s0_x86
	rm -rf build
	-$(MAKE) -C povmem clean

.PHONY: all arm sim test bench ko deps deps-lz4 clean
