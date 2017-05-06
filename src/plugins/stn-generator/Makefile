#*****************************************************************************
#         Makefile Build System for Fawkes: stn-generator Plugin
#                            -------------------
#   Created on Sat May  6 20:16:21 2017
 
#   Copyright (C) 2017 by Matthias Loebach
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

LIBS_stn_generator = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface gvc cgraph

OBJS_stn_generator = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

PLUGINS_all = $(PLUGINDIR)/stn-generator.$(SOEXT)

OBJS_all = $(OBJS_stn-generator)

include $(BUILDSYSDIR)/base.mk
