#*****************************************************************************
#              Makefile Build System for Fawkes: Geometry Library
#                            -------------------
#   Created on Sun Oct 14 18:48:54 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/gui.mk
include $(BUILDSYSDIR)/ext/gmsl

CFLAGS  += $(CFLAGS_GUI)
LDFLAGS += $(LDFLAGS_GUI)

LIBS_libfawkesguiutils = stdc++ m fawkescore fawkesnetcomm fawkesblackboard \
			 fawkesinterface fawkesplugin fawkeslogging fawkesnetworklogger fawkesutils
OBJS_libfawkesguiutils = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
HDRS_libfawkesguiutils = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h))

OBJS_all = $(OBJS_libfawkesguiutils)

ifeq ($(HAVE_GUI),1)
  LIBS_gui  = $(LIBDIR)/libfawkesguiutils.so
else
  WARN_TARGETS += warning_guiutils
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_guiutils:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building guiutils$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk
