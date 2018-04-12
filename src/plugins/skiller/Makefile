#*****************************************************************************
#       Makefile Build System for Fawkes: Skill Execution Runtime Plugin
#                            -------------------
#   Created on Fri Feb 08 00:47:23 2008
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
include $(BUILDSYSDIR)/lua.mk
include $(LIBSRCDIR)/utils/utils.mk
include $(LIBSRCDIR)/tf/tf.mk
include $(LIBSRCDIR)/navgraph/navgraph.mk

PRESUBDIRS = interfaces
SUBDIRS = consoletool rest-api

# Uncomment to enable time tracking for the skiller
# CFLAGS += -DSKILLER_TIMETRACKING -DSKILLER_TT_MOD=50
CFLAGS  += $(CFLAGS_LUA)

ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
endif

LIBS_skiller = fawkescore fawkesutils fawkesaspects fawkesnetcomm fawkeslua \
	       fawkesblackboard fawkesinterface fawkeslogging \
	       SkillerInterface SkillerDebugInterface

ifeq ($(HAVE_NAVGRAPH),1)
  LIBS_skiller += fawkesnavgraphaspect
  CFLAGS += -DHAVE_NAVGRAPH
else
  FILTER_OUT += skiller_navgraph_feature.o
  WARN_TARGETS += warning_navgraph
endif

OBJS_skiller = $(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))
OBJS_all    = $(OBJS_skiller)

ifeq ($(HAVE_LUA),1)
  ifeq ($(HAVE_TOLUA),1)
    PLUGINS_all = $(PLUGINDIR)/skiller.so
  else
    WARN_TARGETS += warning_tolua
  endif
else
  WARN_TARGETS += warning_lua
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_lua warning_tolua
warning_lua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Skiller Plugin$(TNORMAL) (Lua >= $(LUA_VERSION) not installed)"

warning_tolua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Skiller Plugin$(TNORMAL) (tolua++[-devel] not installed)"

warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Navgraph feature for Skiller Plugin$(TNORMAL) (navgraphaspect not available)"
endif

include $(BUILDSYSDIR)/base.mk

