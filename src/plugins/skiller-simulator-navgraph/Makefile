#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Tue 07 Jan 2020 16:04:34 CET
#   Copyright (C) 2020 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

LIBS_skiller_simulator_navgraph = fawkescore fawkesutils fawkesaspects \
                                  fawkesblackboard fawkesinterface \
                                  fawkeslogging fawkes_skiller_time_estimator
OBJS_skiller_simulator_navgraph = skiller_simulator_navgraph_plugin.o \
                                  skiller_simulator_navgraph_thread.o \
                                  navgraph_estimator.o

CFLAGS       += $(CFLAGS_CPP17)
OBJS_all      = $(OBJS_skiller_simulator_navgraph)
PLUGINS_all   = $(PLUGINDIR)/skiller-simulator-navgraph.so
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
