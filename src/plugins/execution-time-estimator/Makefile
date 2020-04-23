#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Thu 23 Apr 2020 16:53:02 CEST
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
include $(LIBSRCDIR)/utils/utils.mk

LIBS_execution_time_estimator = fawkescore fawkesaspects

OBJS_execution_time_estimator = execution_time_estimator_plugin.o

OBJS_all = $(OBJS_execution_time_estimator)
PLUGINS_all = $(PLUGINDIR)/execution-time-estimator.so
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
