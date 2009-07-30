#*****************************************************************************
#              Makefile Build System for Fawkes: ECLiPSe Config
#                            -------------------
#   Created on Wed Jul 15 15:40:56 2009
#   copyright (C) 2009 by Daniel Beck, AllemaniACs RoboCup Team
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

ECLIPSE_BINARY = eclipse

HAVE_ECLIPSE = $(if $(shell which $(ECLIPSE_BINARY) > /dev/null; echo $${?/1/}),1,0)

ifeq ($(HAVE_ECLIPSE),1)
  HOSTARCH = $(shell $(ECLIPSE_BINARY) -e "get_flag(hostarch,A),printf(\"%p\", [A])")
  ifeq ($(findstring $(patsubst i%86,i386,$(ARCH)),$(HOSTARCH)),)
    HAVE_ECLIPSE = 0
  endif

  ECLIPSE_LIBDIR = $(shell $(ECLIPSE_BINARY) -e "get_flag(installation_directory,D),printf(\"%p\", [D])")/lib/$(HOSTARCH)
  ECLIPSE_INCDIR = $(shell $(ECLIPSE_BINARY) -e "get_flag(installation_directory,D),printf(\"%p\", [D])")/include/$(HOSTARCH)

  ECLIPSE_CFLAGS = -I$(ECLIPSE_INCDIR) -DECLIPSE_CODE_DIR=\"$(abspath $(BASEDIR)/src/plugins/readylogagent)\"
  ECLIPSE_LDFLAGS = -L$(ECLIPSE_LIBDIR) -Wl,-R$(ECLIPSE_LIBDIR)
endif