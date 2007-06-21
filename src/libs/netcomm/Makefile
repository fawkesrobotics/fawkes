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

ifneq ($(SRCDIR),)
include $(SRCDIR)/netcomm.mk
else
include netcomm.mk
endif

ifneq ($(HAVE_AVAHI),1)
  WARN_TARGETS += warn_avahi
  OMIT_OBJECTS += dns-sd/%
else
  CFLAGS += $(shell $(PKGCONFIG) --cflags avahi-client)
  LDFLAGS_libnetcomm += $(shell pkg-config --libs avahi-client)
endif
ifneq ($(HAVE_LIBCRYPTO),1)
  ERROR_TARGETS += error_libcrypto
else
  CFLAGS += $(shell $(PKGCONFIG) --cflags $(LIBCRYPTO_PKG))
  LDFLAGS_libnetcomm += $(shell pkg-config --libs $(LIBCRYPTO_PKG))
endif


ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS) $(ERROR_TARGETS)
.PHONY: warn_avahi
warn_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting mDNS/DNS-SD support$(TNORMAL) (Avahi not installed)"
.PHONY: error_libcrypto
error_libcrypto:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)netcomm cannot be built$(TNORMAL) (No OpenSSL/libcrypto installed)"
	$(SILENT)exit 1
endif

LIBS_libnetcomm = core utils
OBJS_libnetcomm = $(filter-out $(OMIT_OBJECTS),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))

OBJS_all = $(OBJS_libnetcomm)
LIBS_all  = $(LIBDIR)/libnetcomm.so

include $(BASEDIR)/etc/buildsys/base.mk

