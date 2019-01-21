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
include $(BUILDSYSDIR)/libxmlpp.mk

LIBS_refboxcomm = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkesnetcomm \
		  GameStateInterface SwitchInterface
OBJS_refboxcomm = $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))) \
                  processor/processor.o processor/state_handler.o processor/remotebb.o
OBJS_all    = $(OBJS_refboxcomm)
PLUGINS_all = $(PLUGINDIR)/refboxcomm.so
PLUGINS_build = $(PLUGINS_all)

ifeq ($(HAVE_LIBXMLPP),1)
  CFLAGS  += -DHAVE_MSL2010 $(CFLAGS_LIBXMLPP)
  LDFLAGS += $(LDFLAGS_LIBXMLPP)
  OBJS_refboxcomm += processor/msl2010.o
else
  WARN_TARGETS += warning_libxmlpp
endif

ifeq ($(HAVE_CPP11),1)
  CFLAGS += -DHAVE_SPL
  LIBS_refboxcomm += SoccerPenaltyInterface
  OBJS_refboxcomm += processor/spl.o
else
  WARN_TARGETS += warning_spl_cpp11
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libxmlpp warning_spl_cpp11
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting MSL2010 support$(TNORMAL) (libxml++[-devel] not installed)"
warning_sbl_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting SPL support$(TNORMAL) (C++11 not available)"
endif

include $(BUILDSYSDIR)/base.mk
