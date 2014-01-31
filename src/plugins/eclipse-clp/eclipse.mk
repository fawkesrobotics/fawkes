#*****************************************************************************
#              Makefile Build System for Fawkes: ECLiPSe Config
#                            -------------------
#   Created on Wed Jul 15 15:40:56 2009
#   Copyright (C) 2009 by Daniel Beck, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BASEDIR)/etc/buildsys/config.mk
include $(abspath $(BUILDSYSDIR)/ext/gmsl)

ECLIPSE_BINARY = eclipse-clp
ECLIPSE_MIN_VERSION_MAJOR=6
ECLIPSE_MIN_VERSION_MINOR=0

HAVE_ECLIPSE = $(if $(shell which $(ECLIPSE_BINARY) >/dev/null 2>&1; echo $${?/1/}),1,0)

ifeq ($(HAVE_ECLIPSE),1)
  HOSTARCH=$(shell $(ECLIPSE_BINARY) -e "get_flag(hostarch,A),printf(\"%p\", [A])")
  ifeq ($(findstring $(patsubst i%86,i386,$(ARCH)),$(HOSTARCH)),)
    HAVE_ECLIPSE=0
    ECLIPSE_FAIL_REASON="system architecture not supported"
  endif
  ECLIPSE_VERSION=$(shell $(ECLIPSE_BINARY) -e "get_flag(version,A),printf(\"%p\", [A])")
  ECLIPSE_VERSION_SPLITTED=$(call split,.,$(ECLIPSE_VERSION))
  ECLIPSE_VERSION_MAJOR=$(word 1,$(ECLIPSE_VERSION_SPLITTED))
  ECLIPSE_VERSION_MINOR=$(word 2,$(ECLIPSE_VERSION_SPLITTED))
  ifneq ($(call gte,$(ECLIPSE_VERSION_MAJOR),$(ECLIPSE_MIN_VERSION_MAJOR)),$(true))
    HAVE_ECLIPSE=0
    ECLIPSE_FAIL_REASON="Insufficient ECLiPSe version, installed $(ECLIPSE_VERSION), required $(ECLIPSE_MIN_VERSION_MAJOR).$(ECLIPSE_MIN_VERSION_MINOR)"
  else
    ifeq ($(ECLIPSE_VERSION_MAJOR),$(ECLIPSE_MIN_VERSION_MAJOR))
      ifneq ($(call gte,$(ECLIPSE_VERSION_MINOR),$(ECLIPSE_MIN_VERSION_MINOR)),$(true))
        HAVE_ECLIPSE=0
        ECLIPSE_FAIL_REASON="Insufficient ECLiPSe version, installed $(ECLIPSE_VERSION), required $(ECLIPSE_MIN_VERSION_MAJOR).$(ECLIPSE_MIN_VERSION_MINOR)"
      endif
    endif
  endif
else
  ECLIPSE_FAIL_REASON="ECLiPSe not installed"
endif

ifeq ($(HAVE_ECLIPSE),1)
  ifeq ( $(shell pkg-config --exists eclipse-clp), 0 )  
    ECLIPSE_LIB = $(shell pkg-config --libs eclipse-clp)
    ECLIPSE_INC = $(shell pkg-config --cflags eclipse-clp)
  
    ECLIPSE_CFLAGS = $(ECLIPSE_INC) -DECLIPSE_CODE_DIR=\"$(abspath $(BASEDIR)/src/plugins/eclipse-clp)\"
    ECLIPSE_LDFLAGS = $(ECLIPSE_LIB)
  else
    ECLIPSE_LIBDIR = $(shell $(ECLIPSE_BINARY) -e "get_flag(installation_directory,D),printf(\"%p\", [D])")/lib/$(HOSTARCH)
    ECLIPSE_INCDIR = $(shell $(ECLIPSE_BINARY) -e "get_flag(installation_directory,D),printf(\"%p\", [D])")/include/$(HOSTARCH)

    ECLIPSE_CFLAGS = -I$(ECLIPSE_INCDIR) -DECLIPSE_CODE_DIR=\"$(abspath $(BASEDIR)/src/plugins/eclipse-clp)\"
    ECLIPSE_LDFLAGS = -L$(ECLIPSE_LIBDIR) -Wl,-R$(ECLIPSE_LIBDIR)
  endif
endif


