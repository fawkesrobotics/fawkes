#*****************************************************************************
#         Makefile Build System for Fawkes: Syncpoint Test Emitter Plugin
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

LIBS_syncpoint_test_emitter = fawkescore fawkesutils fawkesaspects fvutils \
	        fawkesinterface fawkesblackboard fawkessyncpoint
OBJS_syncpoint_test_emitter = syncpoint_test_emitter_plugin.o \
					syncpoint_test_emitter_thread.o

OBJS_all    = $(OBJS_syncpoint_test_emitter)

PLUGINS_test = $(PLUGINDIR)/syncpoint_test_emitter.$(SOEXT)


include $(BUILDSYSDIR)/base.mk
