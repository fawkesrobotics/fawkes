#*****************************************************************************
#              Makefile Build System for Fawkes: ReadylogAgent Plugin
#                            -------------------
#   Created on Wed Jul 15 11:40:13 2009
#   copyright (C) 2009 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/plugins/readylogagent/eclipse.mk

CFLAGS += $(ECLIPSE_CFLAGS)
LDFLAGS += $(ECLIPSE_LDFLAGS)

LIBS_readylogagent = fawkesaspects eclipse TestInterface
OBJS_readylogagent = plugin.o				\
		     control_thread.o			\
		     eclipse_thread.o			\
		     externals/fawkes_bb_interface.o	\
		     externals/fawkes_logger.o

OBJS_all = $(OBJS_readylogagent)

ifeq ($(HAVE_ECLIPSE),1)
  PLUGINS_all = $(PLUGINDIR)/readylogagent.so
else
  WARN_TARGETS += warning_eclipse
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_eclipse
warning_eclipse:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Readylog Agent Plugin$(TNORMAL) (ECLiPSe not installed)"
endif

include $(BUILDSYSDIR)/base.mk
