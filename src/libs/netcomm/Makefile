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
ifneq ($(PKGCONFIG),)
  HAVE_AVAHI = $(if $(shell $(PKGCONFIG) --print-errors --errors-to-stdout --exists 'avahi-client'),0,1)
  HAVE_CRYPTO = $(if $(shell $(PKGCONFIG) --print-errors --errors-to-stdout --exists 'libcrypto'),0,1)
endif
ifneq ($(HAVE_AVAHI),1)
ERROR_TARGETS += error_avahi
endif
ifneq ($(HAVE_CRYPTO),1)
ERROR_TARGETS += error_libcrypto
endif

#ifneq ($(SRCDIR),)
all: $(ERROR_TARGETS)
.PHONY: error_avahi
error_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)netcomm cannot be built$(TNORMAL) (No Avahi installed)"
	$(SILENT)exit 1
.PHONY: error_libcrypto
error_libcrypto:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)netcomm cannot be built$(TNORMAL) (No OpenSSL/libcrypto installed)"
	$(SILENT)exit 1
#endif

CFLAGS = -g $(shell $(PKGCONFIG) --cflags avahi-client) \
	 $(shell $(PKGCONFIG) --cflags libcrypto)

LIBS_libnetcomm = core utils
LDFLAGS_libnetcomm = $(shell pkg-config --libs avahi-client) \
		     $(shell pkg-config --libs libcrypto)
OBJS_libnetcomm = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))

OBJS_all = $(OBJS_libnetcomm)
LIBS_all  = $(LIBDIR)/libnetcomm.so

include $(BASEDIR)/etc/buildsys/base.mk

