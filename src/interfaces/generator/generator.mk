#*****************************************************************************
#            Makefile Build System for Fawkes : Interfaces Generator
#                            -------------------
#   Created on Fri Feb 01 11:37:25 2008
#   copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLPP = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
endif

BUILD_INTERFACE_GENERATOR=$(HAVE_LIBXMLPP)

ifeq ($(BUILD_INTERFACE_GENERATOR),1)
  CFLAGS += $(shell $(PKGCONFIG) --cflags libxml++-2.6)
endif

