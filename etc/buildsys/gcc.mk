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

# Check für C++0x/C++11 availability
ifeq ($(call gcc_atleast_version,4,3),1)
  HAVE_CPP11=1
  CFLAGS_CPP11=-std=c++0x
  ifeq ($(call gcc_atleast_version,4,7),1)
    CFLAGS_CPP11=-std=c++11
  endif
endif

CFLAGS_MTUNE_NATIVE=-march=native -mtune=native

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

endif # __buildsys_gcc_mk_

