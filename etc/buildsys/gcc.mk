#*****************************************************************************
#          Makefile Build System for Fawkes: GCC settings and flags
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

ifndef __buildsys_gcc_mk_
__buildsys_gcc_mk_ := 1

### GCC version information
GCC_VERSION=$(shell LANG=C $(CC) -v 2>&1 | grep "gcc version" | awk '{ print $$3 }')
GCC_VERSION_SPLITTED=$(call split,.,$(GCC_VERSION))
GCC_VERSION_MAJOR=$(word 1,$(GCC_VERSION_SPLITTED))
GCC_VERSION_MINOR=$(word 2,$(GCC_VERSION_SPLITTED))

# Use with:
# ifeq ($(call gcc_atleast_version,M,N),1)
# ...
# endif
# where M.N is the minimum requires GCC version
gcc_atleast_version = $(strip $(if $(call gt,$(GCC_VERSION_MAJOR),$1),1,	\
                         $(if $(call eq,$(GCC_VERSION_MAJOR),$1),		\
                           $(if $(call gte,$(GCC_VERSION_MINOR),$2),1))))

gcc_lessthan_version = $(strip $(if $(call lt,$(GCC_VERSION_MAJOR),$1),1,	\
                         $(if $(call eq,$(GCC_VERSION_MAJOR),$1),		\
                           $(if $(call lt,$(GCC_VERSION_MINOR),$2),1))))

# Check für newer C++ standards availability
# According to https://gcc.gnu.org/projects/cxx-status.html
ifeq ($(call gcc_atleast_version,4,3),1)
  HAVE_CPP11=1
  CFLAGS_CPP11=-std=c++0x
endif
ifeq ($(call gcc_atleast_version,4,7),1)
  CFLAGS_CPP11=-std=c++11
endif
ifeq ($(call gcc_atleast_version,4,8),1)
  HAVE_CPP14=1
  CFLAGS_CPP14=-std=c++1y
endif
ifeq ($(call gcc_atleast_version,5,0),1)
  HAVE_CPP17=1
  CFLAGS_CPP14=-std=c++14
  CFLAGS_CPP17=-std=c++1z
endif
ifeq ($(call gcc_atleast_version,6,0),1)
  # The default for GCC 6 is C++14
  # Reset flags, also CPP11 to avoid downgrade to C++11.
  CFLAGS_CPP11=
  CFLAGS_CPP14=
endif

ifeq ($(call gcc_lessthan_version,4,7),1)
  # Older GCC version can screw up CPU identification when running in QEMU KVM
  IS_QEMU=
  ifeq ($(OS),FreeBSD)
    ifneq ($(findstring QEMU,$(shell sysctl hw.model)),)
      IS_QEMU=1
    endif
  endif
  ifeq ($(OS),Linux)
    ifneq ($(findstring QEMU,$(shell grep "^model name" /proc/cpuinfo)),)
      IS_QEMU=1
    endif
  endif

  ifeq ($(IS_QEMU),1)
    CFLAGS_MTUNE_NATIVE=$(shell gcc -march=native -E -v - </dev/null 2>&1 | sed -n 's/.* -v - //p' | sed -e 's/-march=[^ ]*//g' -e 's/-mbmi//g') \
			-mno-sse3 -mno-ssse3 -mno-sse4.1 -mno-sse4.2 -mno-avx
  else
    CFLAGS_MTUNE_NATIVE=-march=native -mtune=native
  endif
else
  CFLAGS_MTUNE_NATIVE=-march=native -mtune=native
endif

OPENMP_LIBRARY = gomp
CFLAGS_OPENMP  = -fopenmp
LDFLAGS_OPENMP = -l$(OPENMP_LIBRARY)

ifeq ($(USE_OPENMP),1)
  CFLAGS_MINIMUM  += $(CFLAGS_OPENMP)
  LDFLAGS_MINIMUM += $(LDFLAGS_OPENMP)
endif

# Get rid of some annoying (useless) warnings on Raspberry Pi
ifeq ($(ARCH),armv6l)
  CFLAGS_MINIMUM += -Wno-psabi
endif

CFLAG_W_NO_UNUSED_LOCAL_TYPEDEFS=-Wno-unused-local-typedefs

ifeq ($(call gcc_atleast_version,4,6),1)
  HAVE_CPP11_RANGE_FOR=1
endif

endif # __buildsys_gcc_mk_

