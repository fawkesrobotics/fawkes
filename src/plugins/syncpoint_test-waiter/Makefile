#*****************************************************************************
#         Makefile Build System for Fawkes: Syncpoint Test Waiter Plugin
#                            -------------------
#   Created on Wed Jan 08 17:12:13 2014
#   Copyright 2014 by Till Hofmann
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

LIBS_syncpoint_test_waiter = fawkescore fawkesutils fawkesaspects fvutils \
	        fawkesinterface fawkesblackboard fawkessyncpoint
OBJS_syncpoint_test_waiter = syncpoint_test_waiter_plugin.o syncpoint_test_waiter_thread.o

OBJS_all    = $(OBJS_syncpoint_test_waiter)

PLUGINS_test = $(PLUGINDIR)/syncpoint_test_waiter.$(SOEXT)


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_foo
warning_foo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gensis libs" \
                         "$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
