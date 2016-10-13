#*****************************************************************************
#         Makefile Build System for Fawkes: pddl_robot_memory Plugin
#                            -------------------
#   Created on Thu Oct 13 13:34:05 2016
 
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_pddl_robot_memory = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface

OBJS_pddl_robot_memory = pddl_robot_memory_plugin.o pddl_robot_memory_thread.o

PLUGINS_all = $(PLUGINDIR)/pddl_robot_memory.$(SOEXT)

OBJS_all = $(OBJS_pddl_robot_memory)

include $(BUILDSYSDIR)/base.mk