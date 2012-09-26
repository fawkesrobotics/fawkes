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

ifeq ($(call gte,$(GCC_VERSION_MAJOR),4),$(true))
  ifeq ($(call gte,$(GCC_VERSION_MINOR),3),$(true))
    HAVE_CPP11=1
    CFLAGS_CPP11=-std=c++0x
    ifeq ($(call gte,$(GCC_VERSION_MINOR),7),$(true))
      CFLAGS_CPP11=-std=c++11
    endif
  endif
endif

ifeq ($(USE_OPENMP),1)
  CFLAGS_OPENMP  = -fopenmp
  LDFLAGS_OPENMP = -lgomp

  CFLAGS_MINIMUM  += $(CFLAGS_OPENMP)
  LDFLAGS_MINIMUM += $(LDFLAGS_OPENMP)
endif

endif # __buildsys_gcc_mk_

