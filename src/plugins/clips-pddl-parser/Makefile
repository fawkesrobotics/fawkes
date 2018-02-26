#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Fri 16 Feb 2018 17:33:33 CET
#   Copyright (C) 2018 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
include $(BUILDSYSDIR)/clips.mk

LIBS_clips_pddl_parser = fawkescore fawkesclipsaspect fawkespddl_parser
OBJS_clips_pddl_parser = clips_pddl_parser_plugin.o clips_pddl_parser_thread.o \
												 clips_pddl_parser_feature.o \
												 precondition_visitor.o effect_visitor.o

OBJS_all = $(OBJS_clips_pddl_parser)

ifeq ($(HAVE_CLIPS),1)
  CFLAGS += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-pddl-parser.$(SOEXT)
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-pddl-parser plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
