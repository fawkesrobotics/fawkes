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

HAVE_LIBXMLPP   = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
HAVE_SPL        = $(if $(filter nao%,$(BUILD_TYPE)),1)

LIBS_refboxcomm = fawkescore fawkesutils fawkesaspects \
		  GameStateInterface SwitchInterface
OBJS_refboxcomm = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp)))) \
		  processor/processor.o processor/state_handler.o \
		  processor/remotebb.o
OBJS_all = $(OBJS_refboxcomm)
PLUGINS_all = $(PLUGINDIR)/refboxcomm.so

ifeq ($(HAVE_LIBXMLPP),1)
  CFLAGS  += -DHAVE_MSL2010 $(shell $(PKGCONFIG) --cflags libxml++-2.6)
  LDFLAGS += $(shell $(PKGCONFIG) --libs libxml++-2.6)
  OBJS_refboxcomm += processor/msl2010.o
else
  WARN_TARGETS += warning_libxmlpp
endif

ifeq ($(HAVE_SPL),1)
  CFLAGS += -DHAVE_SPL
  LIBS_refboxcomm += SoccerPenaltyInterface
  OBJS_refboxcomm += processor/spl.o
else
  WARN_TARGETS += warning_spl
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libxmlpp warning_spl
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting MSL2010 support$(TNORMAL) (libxml++[-devel] not installed)"
warning_spl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting SPL support$(TNORMAL) (not building in nao environment)"

endif


include $(BUILDSYSDIR)/base.mk
