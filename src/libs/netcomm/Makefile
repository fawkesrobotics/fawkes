#*****************************************************************************
#               Makefile Build System for Fawkes: NetComm Library
#                            -------------------
#   Created on Tue Nov 07 16:43:45 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

ifneq ($(HAVE_AVAHI),1)
  WARN_TARGETS += warn_avahi
  OMIT_OBJECTS += dns-sd/%
else
  CFLAGS += $(CFLAGS_AVAHI)
  LDFLAGS_libfawkesnetcomm += $(LDFLAGS_AVAHI)
endif
ifneq ($(HAVE_LIBCRYPTO),1)
  WARN_TARGETS += warning_libcrypto
else
  CFLAGS += $(shell $(PKGCONFIG) --cflags $(LIBCRYPTO_PKG))
  LDFLAGS_libfawkesnetcomm += $(shell $(PKGCONFIG) --libs $(LIBCRYPTO_PKG))
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(ERROR_TARGETS)
.PHONY: warn_avahi
warn_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting mDNS/DNS-SD support$(TNORMAL) (Avahi not installed)"
.PHONY: warning_libcrypto
warning_libcrypto:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting encryption support$(TNORMAL) (OpenSSL/libcrypto not found)"
endif

LIBS_libfawkesnetcomm = stdc++ m fawkescore fawkesutils
OBJS_libfawkesnetcomm = $(filter-out $(OMIT_OBJECTS),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
HDRS_libfawkesnetcomm = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h $(SRCDIR)/*/*/*.h))

OBJS_all = $(OBJS_libfawkesnetcomm)
LIBS_all  = $(LIBDIR)/libfawkesnetcomm.so

include $(BUILDSYSDIR)/base.mk

