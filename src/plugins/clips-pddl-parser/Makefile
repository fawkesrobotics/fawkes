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
include $(BUILDSYSDIR)/pddl_parser.mk

LIBS_clips_pddl_parser = fawkescore fawkesclipsaspect
OBJS_clips_pddl_parser = clips_pddl_parser_plugin.o clips_pddl_parser_thread.o \
												 clips_pddl_parser_feature.o \
												 precondition_visitor.o effect_visitor.o

OBJS_all    = $(OBJS_clips_pddl_parser)
PLUGINS_all = $(PLUGINDIR)/clips-pddl-parser.$(SOEXT)

ifeq ($(HAVE_CLIPS)$(HAVE_PDDL_PARSER),11)
  CFLAGS += $(CFLAGS_CLIPS) $(CFLAGS_PDDL_PARSER)
  LDFLAGS += $(LDFLAGS_CLIPS) $(LDFLAGS_PDDL_PARSER)

  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS += warning_clips
  endif
  ifneq ($(HAVE_PDDL_PARSER),1)
    WARN_TARGETS += warning_pddl_parser
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-pddl-parser plugin$(TNORMAL) ($(CLIPS_ERROR))"
warning_pddl_parser:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-pddl-parser plugin$(TNORMAL) (pddl_parser not found)"
endif

include $(BUILDSYSDIR)/base.mk
