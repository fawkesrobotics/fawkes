#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Fri 13 Oct 2017 13:36:15 CEST
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

BASEDIR=../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/boost.mk

LIBS_libfawkespddl_parser = fawkescore
OBJS_libfawkespddl_parser = $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))
HDRS_libfawkespddl_parser = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all += $(OBJS_libfawkespddl_parser)

ifeq ($(HAVE_BOOST),1)
  LIBS_all += $(LIBDIR)/libfawkespddl_parser.so
else
  WARN_TARGETS += warning_boost
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_boost
warning_boost:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PDDL parser library$(TNORMAL) (boost not available)"
endif

include $(BUILDSYSDIR)/base.mk
