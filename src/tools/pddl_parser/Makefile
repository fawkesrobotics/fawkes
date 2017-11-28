#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Fri 13 Oct 2017 14:51:14 CEST
#   Copyright (C) 2017 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
include $(LIBSRCDIR)/pddl_parser/pddl_parser.mk

LIBS_pddl_parser = stdc++ fawkescore fawkespddl_parser
OBJS_pddl_parser = main.o
OBJS_all = $(OBJS_pddl_parser)

ifeq ($(HAVE_PDDL_PARSER),1)
  BINS_all += $(BINDIR)/pddl_parser
else
	WARN_TARGETS += warning_pddl_parser
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_pddl_parser
warning_pddl_parser:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PDDL parser$(TNORMAL) (lib pddl_parser not available)"
endif

include $(BUILDSYSDIR)/base.mk
