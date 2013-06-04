#*****************************************************************************
#              Makefile Build System for Fawkes: Kinova Plugin
#                            -------------------
#   Created on Tue Jun 04 13:13:20 2013
#   Copyright (C) 2013 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

#Check for libusb-1.0
ifneq ($(PKGCONFIG),)
  HAVE_LIBUSB = $(if $(shell $(PKGCONFIG) --exists 'libusb-1.0'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBUSB), 1)
  CFLAGS_LIBUSB  = $(shell $(PKGCONFIG) --cflags 'libusb-1.0')
  LDFLAGS_LIBUSB = $(shell $(PKGCONFIG) --libs 'libusb-1.0')
endif