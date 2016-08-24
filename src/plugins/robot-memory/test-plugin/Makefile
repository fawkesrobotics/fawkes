#*****************************************************************************
#         Makefile Build System for Fawkes: robot_memory_test Plugin
#                            -------------------
#   Created on Wed Aug 24 13:37:27 2016
 
#   Copyright (C) 2016 by Frederik Zwilling
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/gtest.mk

LIBS_robot_memory_test = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface

OBJS_robot_memory_test = robot_memory_test_plugin.o robot_memory_test_thread.o

PLUGINS_all = $(PLUGINDIR)/robot_memory_test.$(SOEXT)

OBJS_all = $(OBJS_robot_memory_test)

include $(BUILDSYSDIR)/base.mk