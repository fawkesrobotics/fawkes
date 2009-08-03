#*****************************************************************************
#       Makefile Build System for Fawkes: RefBox Communication Plugin
#                            -------------------
#   Created on Sun Apr 19 13:04:31 2009 (on way to German Open 2009)
#   Copyright (C) 2009 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/download.mk

HAVE_LIBXMLPP   = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)

LIBS_refboxcomm = fawkescore fawkesutils fawkesaspects GameStateInterface SplPenaltyInterface
OBJS_refboxcomm = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))
OBJS_all = $(OBJS_refboxcomm)

ifeq ($(HAVE_LIBXMLPP),1)
  PLUGINS_all = $(PLUGINDIR)/refboxcomm.so
  CFLAGS  += $(shell $(PKGCONFIG) --cflags libxml++-2.6)
  LDFLAGS += $(shell $(PKGCONFIG) --libs libxml++-2.6)
else
  WARN_TARGETS += warning_libxmlpp
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libxmlpp
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting refboxcomm plugin$(TNORMAL) (libxml++[-devel] not installed)"

endif


include $(BASEDIR)/etc/buildsys/base.mk
