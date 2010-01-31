#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings when installed sys-wide
#                            -------------------
#   Created on Sun Jan 31 12:15:05 2010
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

BUILDCONFDIR       = $(BUILDSYSDIR)/conf
INSTALL_PREFIX     = /usr/local
INSTALL_RESDIR     = $(INSTALL_PREFIX)/share/fawkes
INSTALL_INCDIR     = $(INSTALL_PREFIX)/include/fawkes

# For x86_64 we must append 64 to lib dir
LIBBITS=$(if $(call seq,$(ARCH),x86_64),64)

DEFAULT_INCLUDES  += -I$(abspath $(INSTALL_INCDIR))
LIBDIRS_BASE      += $(INSTALL_PREFIX)/lib$(LIBBITS)/fawkes/interfaces
