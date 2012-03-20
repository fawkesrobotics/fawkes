#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings specific to Nao Cross
#                            -------------------
#   Created on Tue Jul 01 11:14:43 2008
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before rules.mk)
endif

include $(BUILDSYSDIR)/btypes/config_fawkes.mk

# It's an AMD Geode
TARGET_ARCH=i586

# Use Nao cross compilation environment
OE_PATH = /opt/oe-nao/tmp

SYSROOT=$(OE_PATH)/staging/geode-linux
CROSSROOT=$(OE_PATH)/cross/geode

TARGET_BASEDIR=/opt/fawkes
TARGET_NAOQIROOT=/home/nao/naoqi

EXEC_BASEDIR=$(TARGET_BASEDIR)

CC = $(CROSSROOT)/bin/$(TARGET_ARCH)-linux-gcc
CXX=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-g++
AR=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-ar
RANLIB=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-ranlib
LD=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-gcc
STRIP=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-strip
OBJCOPY=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-objcopy
OBJDUMP=$(CROSSROOT)/bin/$(TARGET_ARCH)-linux-objdump

PKGCONFIG = PKG_CONFIG_SYSROOT_DIR=$(SYSROOT) PKG_CONFIG_PATH=$(SYSROOT)/usr/lib/pkgconfig \
	    PKG_CONFIG_LIBDIR=$(PKG_CONFIG_PATH) LD_LIBRARY_PATH=$(SYSROOT)/usr/lib \
	    $(OE_PATH)/staging/$(ARCH)-linux/usr/bin/pkg-config

# For an optimized compile replace "-g" with "-O3 -mtune=geode -march=geode"
# You can also use both, -g and -O3. This will give you almost the optimized
# performance and almost all the debugging info. Often sufficient.
#
# NOTE: In newer versions (observed in NaoQi 1.10.37) compiling with anything
# other than -O0 breaks the built and leads to a segfault on string assignment
# when loading plugins. Adding all the flags that are listed in the GCC man page
# for higher optimization levels works, -O1 must add something implicitly that's
# not documented in the man page and thus we cannot disable it.
# Second line contains -O1 flags, following line -O2 flags
CFLAGS_BASE +=	-g -mtune=geode -march=geode \
		-fauto-inc-dec -fcprop-registers -fdce -fdefer-pop -fdse -fguess-branch-probability -fif-conversion2 -fif-conversion -fipa-pure-const -fipa-reference -fmerge-constants -fsplit-wide-types -ftree-ccp -ftree-ch -ftree-copyrename -ftree-dce -ftree-dominator-opts -ftree-dse -ftree-fre -ftree-sra -ftree-ter -funit-at-a-time -finline-small-functions -fomit-frame-pointer -finline-functions-called-once -fdelete-null-pointer-checks -fmove-loop-invariants \
		-fthread-jumps -falign-functions  -falign-jumps -falign-loops  -falign-labels -fcaller-saves -fcrossjumping -fcse-follow-jumps  -fcse-skip-blocks -fdelete-null-pointer-checks -fexpensive-optimizations -fgcse  -fgcse-lm -finline-small-functions -foptimize-sibling-calls -fpeephole2 -fregmove -freorder-blocks -freorder-functions -frerun-cse-after-loop -fsched-interblock  -fsched-spec -fschedule-insns  -fschedule-insns2 -fstrict-aliasing -fstrict-overflow -ftree-pre -ftree-vrp

DEFAULT_INCLUDES += -I$(SYSROOT)/usr/include \
		    -I$(CROSSROOT)/lib/gcc/$(TARGET_ARCH)-linux/4.3.3/include/ \
		    -I$(SYSROOT)/usr/include/c++/ \
		    -I$(SYSROOT)/usr/include/c++/$(TARGET_ARCH)-linux/
LDFLAGS_BASE     += -L$(CROSSROOT)/$(TARGET_ARCH)-linux/lib/ \
		    -L$(CROSSROOT)/$(TARGET_ARCH)-linux/usr/lib/ \
		    -Wl,-R$(CROSSROOT)/$(TARGET_ARCH)-linux/lib/ \
		    -Wl,-R$(SYSROOT)/usr/lib/
CFLAGS_BASE      += --sysroot $(SYSROOT)
LDFLAGS_BASE     += --sysroot $(SYSROOT)
GCC_USE_OPENMP    = 0
CFLAGS_OPENMP     =
LDFLAGS_OPENMP    =


# Deployment settings
NAOQI_MODS = naofawkes

RSYNC = rsync
RSYNC_FLAGS = --no-owner -vdRz --progress --exclude='*.git*' \
		--include '$(subst $(abspath $(BASEDIR))/,,$(CONFDIR))/default.db' \
		--exclude '$(subst $(abspath $(BASEDIR))/,,$(CONFDIR))/*.db' \
		$(addsuffix .so*,$(addprefix --exclude=$(subst $(abspath $(BASEDIR))/,,$(LIBDIR))/lib,$(NAOQI_MODS)))
RSYNC_DIRS = $(addprefix $(BASEDIR)/./,bin cfg etc lib plugins res lib/lua/ src/lua/)
RSYNC_NAOQIMODS_FLAGS = --no-owner -vz --progress
RSYNC_NAOQIMODS = $(addsuffix .so*,$(addprefix $(LIBDIR)/lib,$(NAOQI_MODS)))
RSYNC_USER = nao
RSYNC_HOST = nao-1.local
HOST       = $(RSYNC_HOST)

