#*****************************************************************************
#               Makefile Build System for Fawkes: NetComm Library
#                            -------------------
#   Created on Tue Nov 07 16:43:45 2006
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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
  HAVE_AVAHI     := $(if $(shell $(PKGCONFIG) --exists 'avahi-client'; echo $${?/1/}),1,0)
  HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --exists 'libcrypto'; echo $${?/1/}),1,0)
  LIBCRYPTO_PKG  := libcrypto
  ifneq ($(HAVE_LIBCRYPTO),1)
    HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --exists 'openssl'; echo $${?/1/}),1,0)
    LIBCRYPTO_PKG  := openssl
  endif
endif
ifeq ($(HAVE_AVAHI),1)
  CFLAGS_AVAHI  = -DHAVE_AVAHI $(shell $(PKGCONFIG) --cflags avahi-client)
  LDFLAGS_AVAHI = $(shell $(PKGCONFIG) --libs avahi-client)
endif
ifeq ($(HAVE_LIBCRYPTO),1)
  CFLAGS += -DHAVE_LIBCRYPTO
endif

