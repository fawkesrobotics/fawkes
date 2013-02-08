#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
__buildsys_config_mk_ := 1

BUILDSYSDIR    ?= $(abspath $(BASEDIR)/etc/buildsys)
FAWKES_BASEDIR  = $(BASEDIR)
TOP_BASEDIR     = $(BASEDIR)

SUBMODULE        =
SUBMODULE_INTERN =
SUBMODULE_EXTERN =

# Check if we are a submodule and the secondary buildsys config has
# not yet been included
ifneq ($(wildcard $(BASEDIR)/../.gitmodules),)
  ifneq ($(wildcard $(BASEDIR)/../fawkes),)
    # this fawkes instance is indeed a sub-module!
    SUBMODULE = 1
    SUBMODULE_INTERN=1
    TOP_BASEDIR = $(BASEDIR)/..
  endif
else
  ifneq ($(wildcard $(BASEDIR)/.gitmodules),)
    ifneq ($(wildcard $(BASEDIR)/fawkes),)
      # this fawkes instance is indeed a sub-module!
      SUBMODULE = 1
      SUBMODULE_EXTERN=1
      FAWKES_BASEDIR=$(BASEDIR)/fawkes
    endif
  endif
endif
#$(warning SUBMODULE $(SUBMODULE)  INTERN: $(SUBMODULE_INTERN)   EXTERN: $(SUBMODULE_EXTERN))

include $(BUILDSYSDIR)/ext/gmsl

### Debugging related options
SILENTSYMB = @
SILENT = @
ifeq ($(VERBOSE),1)
SILENT =
endif
ifeq ($(filter uncolored-%,$(MAKECMDGOALS)),)
COLORED = 1
endif

### Build type
ifeq ($(SUBMODULE),1)
  ifneq ($(wildcard $(SECONDARY_BUILDSYSDIR)/buildtype.mk),)
    include $(SECONDARY_BUILDSYSDIR)/buildtype.mk
  else
    ifneq ($(wildcard $(BUILDSYSDIR)/buildtype.mk),)
      include $(BUILDSYSDIR)/buildtype.mk
    endif
  endif
else
  ifneq ($(wildcard $(BUILDSYSDIR)/buildtype.mk),)
    include $(BUILDSYSDIR)/buildtype.mk
  endif
endif

BUILD_TYPE ?= fawkes
ARCH=$(shell uname -m)
OS=$(shell uname -s)
DISTRO=$(shell sed -n "/^ID=/ s/ID=//p" /etc/os-release)


### Directories
SRCDIR       ?= .
OBJDIR        = .objs_$(BUILD_TYPE)
DEPDIR        = $(abspath $(SRCDIR)/.deps_$(BUILD_TYPE))
BINDIR        = $(abspath $(TOP_BASEDIR)/bin)
LIBDIR        = $(abspath $(TOP_BASEDIR)/lib)
CONFDIR       = $(abspath $(TOP_BASEDIR)/cfg)
TMPDIR        = /tmp
PLUGINDIR     = $(abspath $(TOP_BASEDIR)/plugins)
RESDIR        = $(abspath $(TOP_BASEDIR)/res)
LIBSRCDIR     = $(abspath $(FAWKES_BASEDIR)/src/libs)
EXTLIBDIR     = $(abspath $(LIBSRCDIR)/extlib)
IFACEDIR      = $(abspath $(TOP_BASEDIR)/lib/interfaces)
IFACESRCDIR   = $(abspath $(TOP_BASEDIR)/src/libs/interfaces)
LOGDIR        = $(abspath $(TOP_BASEDIR)/log)
DOCDIR        = $(abspath $(FAWKES_BASEDIR)/doc)
MANDIR        = $(abspath $(DOCDIR)/man)
BASESRCDIRS   = $(abspath $(FAWKES_BASEDIR)/src $(TOP_BASEDIR)/src)
LIBSRCDIRS    = $(abspath $(FAWKES_BASEDIR)/src/libs $(TOP_BASEDIR)/src/libs)
BUILDCONFDIR  = $(LIBSRCDIR)

# Paths at execution time, may be different if installed or deployed
TARGET_ARCH   ?= $(ARCH)
EXEC_BASEDIR  ?= $(TOP_BASEDIR)
EXEC_BINDIR    = $(abspath $(EXEC_BASEDIR)/bin)
EXEC_LIBDIR    = $(abspath $(EXEC_BASEDIR)/lib)
EXEC_CONFDIR   = $(abspath $(EXEC_BASEDIR)/cfg)
EXEC_USERDIR   = .fawkes
EXEC_PLUGINDIR = $(abspath $(EXEC_BASEDIR)/plugins)
EXEC_RESDIR    = $(abspath $(EXEC_BASEDIR)/res)
EXEC_IFACEDIR  = $(abspath $(EXEC_BASEDIR)/lib/interfaces)
EXEC_LOGDIR    = $(abspath $(EXEC_BASEDIR)/log)
EXEC_DOCDIR    = $(abspath $(EXEC_BASEDIR)/doc)
EXEC_TMPDIR    = /tmp
EXEC_MANDIR    = $(abspath $(EXEX_DOCDIR)/man)

# Some paths divert in submodule configuration
ifeq ($(SUBMODULE_INTERN),1)
  RESDIR      = $(abspath $(FAWKES_BASEDIR)/res)
  EXEC_RESDIR = $(abspath $(FAWKES_BASEDIR)/res)
  IFACESRCDIR = $(abspath $(FAWKES_BASEDIR)/src/libs/interfaces)
endif

VPATH = $(SRCDIR)
DEPFILE = $(DEPDIR)/$(subst ._,,$(subst /,_,$(subst ..,__,$(subst ./,,$(*D))))_)$(*F)

### Programs used, no trivial stuff like ln, rm, ls as per Makefile manual
CC = gcc
LD = gcc
MOC = $(QTDIR)/bin/moc
DOXYGEN = doxygen
PKGCONFIG = $(shell which pkg-config)
NM=nm
ASCIIDOC=asciidoc
ASCIIDOC_A2X=a2x
ifneq ($(wildcard /bin/bash),)
  SHELL = /bin/bash
else
  ifneq ($(wildcard /usr/local/bin/bash),)
    SHELL = /usr/local/bin/bash
  else
    $(error Only bash is supported as shell, but it cannot be found.)
  endif
endif

FAWKES_VERSION_MAJOR = $(lastword $(shell grep "\#define FAWKES_VERSION_MAJOR" $(LIBSRCDIR)/core/version.h))
FAWKES_VERSION_MINOR = $(lastword $(shell grep "\#define FAWKES_VERSION_MINOR" $(LIBSRCDIR)/core/version.h))
FAWKES_VERSION_MICRO = $(lastword $(shell grep "\#define FAWKES_VERSION_MICRO" $(LIBSRCDIR)/core/version.h))
DEFAULT_SOVER        = $(FAWKES_VERSION_MAJOR).$(FAWKES_VERSION_MINOR).$(FAWKES_VERSION_MICRO)
FAWKES_VERSION       = $(FAWKES_VERSION_MAJOR).$(FAWKES_VERSION_MINOR).$(FAWKES_VERSION_MICRO)

### Features ###
# If gcc is used, enable OpenMP?
USE_OPENMP=0
# Build for 32 Bit, even on a 64 Bit machine
DO_32BIT_BUILD=0
# Warn if a binary has no man page?
WARN_MISSING_MANPAGE=1
# Man page sections to check and build
MANPAGE_SECTIONS = 1 8

COMMA := ,

### CFLAGS, preprocessor, compiler and linker options
LIBDIRS_BASE     = $(LIBDIR) $(LIBDIR)/interfaces
LIBDIRS_EXEC_BASE= $(EXEC_LIBDIR) $(EXEC_LIBDIR)/interfaces
LDFLAGS_RPATH    = $(addprefix -Wl$(COMMA)-rpath -Wl$(COMMA),$(LIBDIRS_EXEC_BASE) $(LIBDIRS_BASE) $(LIBDIRS))
DEFAULT_INCLUDES = $(addprefix -I,$(BASESRCDIRS) $(LIBSRCDIRS))
CFLAGS_DEFS      = -DBINDIR=\"$(EXEC_BINDIR)\" -DLIBDIR=\"$(EXEC_LIBDIR)\" \
		   -DPLUGINDIR=\"$(EXEC_PLUGINDIR)\" -DIFACEDIR=\"$(EXEC_IFACEDIR)\" \
		   -DCONFDIR=\"$(EXEC_CONFDIR)\" -DUSERDIR=\"$(EXEC_USERDIR)\" \
		   -DLOGDIR=\"$(EXEC_LOGDIR)\" -DRESDIR=\"$(EXEC_RESDIR)\" \
		   -DTMPDIR=\"$(EXEC_TMPDIR)\" -DSRCDIR=\"$(abspath $(SRCDIR))\" \
		   -DBUILDTYPE=\"$(BUILD_TYPE)\" -DSOEXT=\"$(SOEXT)\"

CFLAGS_MINIMUM   = -fPIC -pthread $(DEFAULT_INCLUDES) $(CFLAGS_DEFS)
LDFLAGS_MINIMUM  = $(LIBDIRS_BASE:%=-L%) -rdynamic -fPIC -Wl,--no-undefined -lstdc++
CFLAGS_BASE      = $(CFLAGS_MINIMUM) $(CFLAGS_EXTRA)
LDFLAGS_BASE     =  $(LDFLAGS_MINIMUM) $(LDFLAGS_RPATH)
LDFLAGS_SHARED   = -shared
ifeq ($(OS),FreeBSD)
  DEFAULT_INCLUDES += -I/usr/local/include
  LDFLAGS_BASE     += -L/usr/local/lib -lpthread
endif
SOEXT               = so

# Required if BASEDIR != EXEC_BASEDIR
SYS_LD_LIBRARY_PATH:=$(LD_LIBRARY_PATH)
export LD_LIBRARY_PATH= $(call merge,:, $(LIBDIRS_BASE) $(LIBDIRS) $(SYS_LD_LIBRARY_PATH))

# Function to convert file to variable pattern
nametr = $(subst /,_,$(subst -,_,$1))

ifeq ($(COLORED),1)
TBOLDGRAY	= \033[1;30m
TBLUE		= \033[0;34m
TBOLDBLUE	= \033[1;34m
TGREEN		= \033[0;32m
TBOLDGREEN	= \033[1;32m
TBROWN		= \033[0;33m
#TYELLOW	= \033[1;33m
TYELLOW		= \033[0;33m
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
ifneq ($(wildcard $(BUILDSYSDIR)/$(CC).mk),)
  include $(BUILDSYSDIR)/$(CC).mk
endif

### Check if there is a build-type specific configuration
ifneq ($(wildcard $(BUILDSYSDIR)/btypes/config_$(BUILD_TYPE).mk),)
  include $(BUILDSYSDIR)/btypes/config_$(BUILD_TYPE).mk
else
  ifneq ($(SECONDARY_BUILDSYSDIR),)
    ifneq ($(wildcard $(SECONDARY_BUILDSYSDIR)/btypes/config_$(BUILD_TYPE).mk),)
      include $(SECONDARY_BUILDSYSDIR)/btypes/config_$(BUILD_TYPE).mk
    endif
  endif
endif

### If we are in the fawkes sub module include the top config 
ifeq ($(SUBMODULE_INTERN),1)
  include $(TOP_BASEDIR)/etc/buildsys/config.mk
endif


### Check if there is a local config for this directory
ifneq ($(SRCDIR),.)
  ifneq ($(wildcard $(SRCDIR)/$(notdir $(SRCDIR)).mk),)
    include $(SRCDIR)/$(notdir $(SRCDIR)).mk
  endif
else
  ifneq ($(wildcard $(notdir $(CURDIR)).mk),)
    include $(notdir $(CURDIR)).mk
  endif
endif

ifeq ($(DO_32BIT_BUILD),1)
  CFLAGS_BASE  += -m32
  LDFLAGS_BASE += -m32

  SYSLIBDIR32 = /usr/lib$(if $(wildcard /usr/lib32),32)
  PKGCONFIG = PKG_CONFIG_PATH=$(SYSLIBDIR32)/pkgconfig PKG_CONFIG_LIBDIR=$$PKG_CONFIG_PATH; pkg-config
  ARCH=i386
endif

endif # __buildsys_config_mk_

