
#*****************************************************************************
#            Makefile Build System for Fawkes: PanTilt Plugin
#                            -------------------
#   Created on Wed Jun 17 22:18:13 2009
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
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
SUBDIRS = tool

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
endif

# Sub directory name that contain PTU drivers
DRIVERS = dirperc robotis sony

LIBS_pantilt = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	       fawkesinterface PanTiltInterface LedInterface JointInterface \
	       CameraControlInterface SwitchInterface
OBJS_pantilt = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(foreach d,$(DRIVERS),$(SRCDIR)/$d/*.cpp )))))

PLUGINS_all = $(PLUGINDIR)/pantilt.so
OBJS_all = $(OBJS_pantilt)

include $(BUILDSYSDIR)/base.mk

