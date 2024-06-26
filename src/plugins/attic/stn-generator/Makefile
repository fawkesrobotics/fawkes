#*****************************************************************************
#         Makefile Build System for Fawkes: stn-generator Plugin
#                            -------------------
#   Created on Sat May  6 20:16:21 2017
#   Copyright (C) 2017 by Matthias Loebach
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
include $(BUILDSYSDIR)/pddl_parser.mk
include $(FAWKES_BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_stn_generator = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface

OBJS_stn_generator = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_stn_generator)
PLUGINS_all = $(PLUGINDIR)/stn-generator.$(SOEXT)

ifneq ($(PKGCONFIG),)
  HAVE_GRAPHVIZ    = $(if $(shell $(PKGCONFIG) --exists 'libgvc'; echo $${?/1/}),1,0)
  CFLAGS_GRAPHVIZ  = $(shell $(PKGCONFIG) --cflags 'libgvc') -DWITH_CODEGENS
  LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libgvc')
endif

ifeq ($(HAVE_PDDL_PARSER)$(HAVE_MONGODB)$(HAVE_GRAPHVIZ),111)
  CFLAGS  += $(CFLAGS_MONGODB)  $(CFLAGS_GRAPHVIZ) $(CFLAGS_PDDL_PARSER)
  LDFLAGS += $(LDFLAGS_MONGODB) $(LDFLAGS_GRAPHVIZ) $(LDFLAGS_PDDL_PARSER)
  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_PDDL_PARSER),1)
    WARN_TARGETS += warning_pddl_parser
  endif
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_GRAPHVIZ),1)
    WARN_TARGETS += warning_graphviz
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_pddl_parser warning_mongodb warning_graphviz
warning_pddl_parser:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting STN plugin$(TNORMAL) (pddl_parser not available)"
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting STN plugin$(TNORMAL) (mongo-cxx-driver not available)"
warning_graphviz:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting STN plugin$(TNORMAL) (graphviz not available)"
endif

include $(BUILDSYSDIR)/base.mk
