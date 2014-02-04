#*****************************************************************************
#              Makefile Build System for Fawkes: WorldModel Plugin
#                            -------------------
#   Created on Fir Jun 29 11:53:13 2007 (on flight to RoboCup 2007, Atlanta)
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_worldmodel = m fawkescore fawkesutils fawkesaspects fawkestf \
		  fawkesblackboard fawkesinterface fawkesnetcomm \
		  GameStateInterface ObjectPositionInterface
OBJS_worldmodel = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

OBJS_all    = $(OBJS_worldmodel)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF) -Wno-deprecated-declarations
  LDFLAGS += $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/worldmodel.$(SOEXT)
else
  WARN_TARGETS = warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting worldmodel plugin$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk

