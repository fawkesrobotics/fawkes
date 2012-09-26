#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings specific to Fawkes
#                            -------------------
#   Created on Thu Oct 16 20:00:11 2008
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

include $(BUILDSYSDIR)/btypes/config_fawkes.mk

# Not silent, no color
SILENT  = @
COLORED = 0

# Can be enabled via CFLAGS
GCC_USE_OPENMP=0
DO_32BIT_BUILD=0

# For x86_64/ppc64 we must append 64 to lib dir
LIBBITS=$(if $(or $(call seq,$(ARCH),x86_64),$(call seq,$(ARCH),ppc64)),64)

DESTDIR        =
PREFIX         = /usr/local
SYSCONFDIR     = /etc/fawkes

# Base CFLAGS, LDFLAGS
# - use RPATH for interfaces libdir, required for proper linking while still
#   separating the (many) interface libs from the normal libdir and avoiding
#   the need to add an entry per command to /etc/ld.so.conf.d/ (or similar)
CFLAGS_BASE    = $(CFLAGS_MINIMUM)
LDFLAGS_BASE   = $(LDFLAGS_MINIMUM) -Wl,-rpath=$(EXEC_LIBDIR)/fawkes/interfaces

# Note: the install scripts make the assumption that FFLIBDIR, RESDIR and INCDIR
# paths (with EXEC_ prefix) are specific to Fawkes and can be deleted completely
# without interfering with the rest of the system!
FFLIBSUBDIR        = fawkes
EXEC_BASEDIR       = $(abspath $(PREFIX))
EXEC_CONFDIR       = $(SYSCONFDIR)
EXEC_RESDIR        = $(EXEC_BASEDIR)/share/fawkes
EXEC_LIBDIR        = $(EXEC_BASEDIR)/lib$(LIBBITS)
EXEC_FFLIBDIR      = $(EXEC_LIBDIR)/$(FFLIBSUBDIR)
EXEC_PLUGINDIR     = $(EXEC_FFLIBDIR)/plugins
EXEC_IFACEDIR      = $(EXEC_FFLIBDIR)/interfaces
EXEC_INCDIR        = $(EXEC_BASEDIR)/include/fawkes
EXEC_DFILEDIR      = $(EXEC_BASEDIR)/share/applications
EXEC_DOCDIR        = $(EXEC_BASEDIR)/share/doc/fawkes-$(FAWKES_VERSION)
EXEC_MANDIR        = $(EXEC_BASEDIR)/share/man
EXEC_LUADIR        = $(EXEC_RESDIR)/lua
EXEC_LUALIBDIR     = $(EXEC_FFLIBDIR)/lua
EXEC_BUILDSYSDIR   = $(EXEC_RESDIR)/buildsys
EXEC_BUILDCONFDIR  = $(EXEC_BUILDSYSDIR)/conf

ifneq ($(CFLAGS_EXT),)
  CFLAGS_BASE += $(CFLAGS_EXT)
endif
ifneq ($(LDFLAGS_EXT),)
  LDFLAGS_BASE += $(LDFLAGS_EXT)
endif
