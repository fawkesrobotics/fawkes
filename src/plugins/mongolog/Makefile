#*****************************************************************************
#           Makefile Build System for Fawkes: MongoDB Logging Plugin
#                            -------------------
#   Created on Wed Dec 08 22:52:40 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_mongolog = fawkescore fawkesaspects fawkesinterface fawkesmongodbaspect $(LIBS_MONGODB)
OBJS_mongolog = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_mongolog)

ifeq ($(HAVE_MONGODB),1)
  PLUGINS_all = $(PLUGINDIR)/mongolog.so
else
  WARN_TARGETS += warning_mongodb
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoLog Plugin$(TNORMAL) (mongodb[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

