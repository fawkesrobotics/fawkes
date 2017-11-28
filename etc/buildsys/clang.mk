#*****************************************************************************
#          Makefile Build System for Fawkes: clang settings and flags
#                            -------------------
#   Created on Wed Sep 26  15:49:00 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before gcc.mk)
endif

ifndef __buildsys_clang_mk_
__buildsys_clang_mk_ := 1

### clang version information
CLANG_VERSION=$(shell LANG=C $(CC) -v 2>&1 | grep "clang version" | sed -e 's/^.*clang version \([^ ]*\) .*$$/\1/')
CLANG_VERSION_SPLITTED=$(call split,.,$(CLANG_VERSION))
CLANG_VERSION_MAJOR=$(word 1,$(CLANG_VERSION_SPLITTED))
CLANG_VERSION_MINOR=$(word 2,$(CLANG_VERSION_SPLITTED))

clang_atleast_version = $(strip $(if $(call gt,$(CLANG_VERSION_MAJOR),$1),1,	\
			  $(if $(call eq,$(CLANG_VERSION_MAJOR),$1),		\
			    $(if $(call gte,$(CLANG_VERSION_MINOR),$2),1))))

# According to https://clang.llvm.org/cxx_status.html
ifeq ($(call clang_atleast_version,3,3),1)
  HAVE_CPP11=1
  CFLAGS_CPP11=-std=c++11
endif
ifeq ($(call clang_atleast_version,3,4),1)
  HAVE_CPP14=1
  CFLAGS_CPP14=-std=c++14
endif
ifeq ($(call clang_atleast_version,4,0),1)
  HAVE_CPP17=1
  CFLAGS_CPP17=-std=c++1z
endif
ifeq ($(call clang_atleast_version,5,0),1)
  HAVE_CPP17=1
  CFLAGS_CPP17=-std=c++17
endif

CFLAGS_MTUNE_NATIVE=-march=native -mtune=native
ifeq ($(OS),FreeBSD)
  ifneq ($(findstring QEMU,$(shell sysctl hw.model)),)
    # When virtualized, clang picks the wrong architecture with march=native
    CFLAGS_MTUNE_NATIVE=-mtune=native
  endif
endif

ifeq ($(USE_OPENMP),1)
  $(warning clang has no support for OpenMP at this time)
  CFLAGS_OPENMP  =
  LDFLAGS_OPENMP =

  CFLAGS_MINIMUM  += $(CFLAGS_OPENMP)
  LDFLAGS_MINIMUM += $(LDFLAGS_OPENMP)
endif

# unsupported flags (which are supported on GCC)
CFLAG_W_NO_UNUSED_LOCAL_TYPEDEFS=

# no-missing-braces: https://bugs.llvm.org/show_bug.cgi?id=21629
CFLAGS_DISABLE_WARNINGS=-Wno-missing-braces

HAVE_CPP11_RANGE_FOR=1

ifeq ($(HAVE_CPP11),1)
  CFLAGS_MINIMUM+=$(CFLAGS_CPP11)
else
  $(error Your version of clang is too old and does not support c++11)
endif

endif # __buildsys_clang_mk_

