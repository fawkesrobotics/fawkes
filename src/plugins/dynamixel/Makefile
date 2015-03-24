
#*****************************************************************************
#            Makefile Build System for Fawkes: Dynamixel Servo Plugin
#                            -------------------
#   Created on Mon Mar 23 19:50:14 2015
#   Copyright (C) 2006-2015 by Tim Niemueller, Carologistics RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces

CFLAGS += $(CFLAGS_CPP11)

LIBS_dynamixel = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	       fawkesinterface JointInterface DynamixelServoInterface LedInterface
OBJS_dynamixel = dynamixel_plugin.o act_thread.o sensor_thread.o \
		     servo_chain.o driver_thread.o

OBJS_all = $(OBJS_dynamixel)

PLUGINS_all = $(PLUGINDIR)/dynamixel.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

