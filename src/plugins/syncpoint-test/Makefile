#*****************************************************************************
#               Makefile Build System for Fawkes: SyncPoint Test Plugin
#
#   Created on Wed Mar 04 17:57:42 2015
#   Copyright (C) 2015 by Till Hofmann
#
##*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk

DISABLE_OBJS_all_WARNING=1

LIBS_syncpoint_test = fawkescore fawkesutils fawkesaspects
OBJS_syncpoint_test = test_plugin.o test_thread.o

OBJS_test    = $(OBJS_syncpoint_test)

PLUGINS_test = $(PLUGINDIR)/syncpoint-test.$(SOEXT)

CFLAGS += $(CFLAGS_CPP11)


include $(BUILDSYSDIR)/base.mk
