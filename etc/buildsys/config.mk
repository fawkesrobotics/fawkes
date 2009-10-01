#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
__buildsys_config_mk := 1

include $(BASEDIR)/etc/buildsys/ext/gmsl

### Debugging related options
SILENTSYMB = @
SILENT = @
ifeq ($(filter uncolored-%,$(MAKECMDGOALS)),)
COLORED = 1
endif

### Build type
ifneq ($(wildcard $(realpath $(BASEDIR)/etc/buildsys/buildtype.mk)),)
  include $(realpath $(BASEDIR)/etc/buildsys/buildtype.mk)
else
  BUILD_TYPE = fawkes
endif
ARCH=$(shell uname -m)
OS=$(shell uname -s)

### Directories
SRCDIR     ?= .
OBJDIR      = .objs_$(BUILD_TYPE)
DEPDIR      = $(abspath $(SRCDIR)/.deps_$(BUILD_TYPE))
BINDIR      = $(abspath $(BASEDIR)/bin)
LIBDIR      = $(abspath $(BASEDIR)/lib)
CONFDIR     = $(abspath $(BASEDIR)/cfg)
PLUGINDIR   = $(abspath $(BASEDIR)/plugins)
RESDIR      = $(abspath $(BASEDIR)/res)
LIBSRCDIR   = $(abspath $(BASEDIR)/src/libs)
IFACEDIR    = $(abspath $(BASEDIR)/lib/interfaces)
IFACESRCDIR = $(abspath $(BASEDIR)/src/interfaces)

# Paths at execution time, may be different if installed or deployed
TARGET_ARCH   ?= $(ARCH)
EXEC_BASEDIR  ?= $(BASEDIR)
EXEC_BINDIR    = $(abspath $(EXEC_BASEDIR)/bin)
EXEC_LIBDIR    = $(abspath $(EXEC_BASEDIR)/lib)
EXEC_CONFDIR   = $(abspath $(EXEC_BASEDIR)/cfg)
EXEC_PLUGINDIR = $(abspath $(EXEC_BASEDIR)/plugins)
EXEC_RESDIR    = $(abspath $(EXEC_BASEDIR)/res)
EXEC_IFACEDIR  = $(abspath $(EXEC_BASEDIR)/lib/interfaces)

VPATH = $(SRCDIR)
DEPFILE = $(DEPDIR)/$(subst ._,,$(subst /,_,$(subst ..,__,$(subst ./,,$(*D))))_)$(*F)

### Programs used, no trivial stuff like ln, rm, ls as per Makefile manual
CC = gcc
MOC = $(QTDIR)/bin/moc
DOXYGEN = doxygen
PKGCONFIG = $(shell which pkg-config)
NM=nm
ifneq ($(wildcard /bin/bash),)
  SHELL = /bin/bash
else
  ifneq ($(wildcard /usr/local/bin/bash),)
    SHELL = /usr/local/bin/bash
  else
    $(error Only bash is supported as shell, but it cannot be found.)
  endif
endif

### GCC version information, currently unused and thus commented out
#ifeq ($(CC),gcc)
#  GCC_VERSION=$(shell LANG=C $CC -v 2>&1 | grep "gcc version" | awk '{ print $3 }')
#  GCC_VERSION_MAJOR=$(shell LANG=C $(CC) -v 2>&1 | grep "gcc version" | awk '{ print $$3 }' | awk -F. '{ print $$1 }')
#endif

FAWKES_VERSION_MAJOR = $(lastword $(shell grep FAWKES_VERSION_MAJOR $(LIBSRCDIR)/core/version.h))
FAWKES_VERSION_MINOR = $(lastword $(shell grep FAWKES_VERSION_MINOR $(LIBSRCDIR)/core/version.h))
FAWKES_VERSION_MICRO = $(lastword $(shell grep FAWKES_VERSION_MICRO $(LIBSRCDIR)/core/version.h))
DEFAULT_SOVER        = $(FAWKES_VERSION_MAJOR).$(FAWKES_VERSION_MINOR).$(FAWKES_VERSION_MICRO)

### Features ###
# If gcc is used, enable OpenMP?
GCC_USE_OPENMP=0
# Build for 32 Bit, even on a 64 Bit machine
DO_32BIT_BUILD=0

### CFLAGS, preprocessor, compiler and linker options
LIBDIRS_BASE     = $(LIBDIR) $(LIBDIR)/interfaces
LIBDIRS_EXEC_BASE= $(EXEC_LIBDIR) $(EXEC_LIBDIR)/interfaces
LDFLAGS_RPATH    = $(LIBDIRS_EXEC_BASE:%=-Wl,-R%) $(LIBDIRS_BASE:%=-Wl,-R%) $(LIBDIRS:%=-Wl,-R%)
DEFAULT_INCLUDES = -I$(abspath $(BASEDIR)/src) -I$(abspath $(BASEDIR)/src/libs) -I$(abspath $(BASEDIR)/src/firevision)
CFLAGS_DEFS      = -DBINDIR=\"$(EXEC_BINDIR)\" -DLIBDIR=\"$(EXEC_LIBDIR)\" \
		   -DPLUGINDIR=\"$(EXEC_PLUGINDIR)\" \
		   -DCONFDIR=\"$(EXEC_CONFDIR)\" -DIFACEDIR=\"$(EXEC_IFACEDIR)\" \
		   -DRESDIR=\"$(EXEC_RESDIR)\" -DBUILDTYPE=\"$(BUILD_TYPE)\"

CFLAGS_MINIMUM   = -fPIC -pthread $(DEFAULT_INCLUDES) $(CFLAGS_OPENMP) $(CFLAGS_DEFS)
LDFLAGS_MINIMUM  = $(LIBDIRS_BASE:%=-L%) -rdynamic $(LDFLAGS_OPENMP)
CFLAGS_BASE      = $(CFLAGS_MINIMUM)
LDFLAGS_BASE     =  $(LDFLAGS_MINIMUM) $(LDFLAGS_RPATH)
LDFLAGS_SHARED   = -shared
CFLAGS_OPENMP  = $(if $(filter 1,$(firstword $(GCC_USE_OPENMP))),-fopenmp)
LDFLAGS_OPENMP = $(if $(filter 1,$(firstword $(GCC_USE_OPENMP))),-lgomp)
ifeq ($(OS),FreeBSD)
  ifeq ($(wildcard /usr/local/include/strfunc.h),)
    $(error libstrfunc is needed on FreeBSD, install devel/libstrfunc!)
  endif
  DEFAULT_INCLUDES += -I/usr/local/include
  LDFLAGS_BASE     += -L/usr/local/lib -lpthread -lstrfunc
endif

ifeq ($(DO_32BIT_BUILD),1)
  CFLAGS_BASE  += -m32
  LDFLAGS_BASE += -m32

  PKGCONFIG = PKG_CONFIG_PATH=/usr/lib/pkgconfig PKG_CONFIG_LIBDIR=$$PKG_CONFIG_PATH pkg-config
  ARCH=i386
endif

# Required if BASEDIR != EXEC_BASEDIR
export LD_LIBRARY_PATH=$(call merge,:, $(LIBDIRS_BASE) $(LIBDIRS))

ifeq ($(COLORED),1)
TBOLDGRAY	= \033[1;30m
TBLUE		= \033[0;34m
TBOLDBLUE	= \033[1;34m
TGREEN		= \033[0;32m
TBOLDGREEN	= \033[1;32m
TBROWN		= \033[0;33m
TYELLOW		= \033[1;33m
TRED		= \033[0;31m
TBOLDRED	= \033[1;31m
TNORMAL		= \033[0;39m
TBLACKBG	= \033[40m
TREDBG		= \033[41m
TGREENBG	= \033[42m
TORANGEBG	= \033[43m
TBLUEBG		= \033[44m
TMAGENTABG	= \033[45m
TCYANBG		= \033[46m
TGREYBG		= \033[47m
endif

### Check if there are special config files for the chosen compiler
ifneq ($(wildcard $(realpath $(BASEDIR)/etc/buildsys/$(CC).mk)),)
  include $(BASEDIR)/etc/buildsys/$(CC).mk
endif

### Check if there is a build-type specific configuration
ifneq ($(wildcard $(realpath $(BASEDIR)/etc/buildsys/btypes/config_$(BUILD_TYPE).mk)),)
  include $(BASEDIR)/etc/buildsys/btypes/config_$(BUILD_TYPE).mk
endif

### Check if there is a local config for this directory
ifneq ($(SRCDIR),.)
  ifneq ($(wildcard $(realpath $(SRCDIR)/$(notdir $(SRCDIR)).mk)),)
    include $(SRCDIR)/$(notdir $(SRCDIR)).mk
  endif
else
  ifneq ($(wildcard $(realpath $(notdir $(CURDIR)).mk)),)
    include $(notdir $(CURDIR)).mk
  endif
endif

endif # __buildsys_config_mk_

