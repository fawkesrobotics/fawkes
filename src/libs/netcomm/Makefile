#*****************************************************************************
#               Makefile Build System for Fawkes: NetComm Library
#                            -------------------
#   Created on Tue Nov 07 16:43:45 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
ifeq ($(PKGCONFIG),)
$(error No pkg-config installed, you cannot build the NetComm library!)
else
  AVAHI = $(shell $(PKGCONFIG) avahi-client; echo $$?)
  ifeq ($(AVAHI),1)
    $(error avahi-devel not installed on your system, you cannot build the NetComm library!)
  endif
endif

CFLAGS = -g $(shell $(PKGCONFIG) --cflags avahi-client) \
	 $(shell $(PKGCONFIG) --cflags libcrypto)

LIBS_libnetcomm = core utils
LDFLAGS_libnetcomm = $(shell pkg-config --libs avahi-client) \
		     $(shell pkg-config --libs libcrypto)
OBJS_libnetcomm = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))

OBJS_all = $(OBJS_libnetcomm)
LIBS_all  = $(LIBDIR)/libnetcomm.so

include $(BASEDIR)/etc/buildsys/base.mk

