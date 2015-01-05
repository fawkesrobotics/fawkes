#*****************************************************************************
#     Makefile Build System for Fawkes: Static transform publisher plugin
#                            -------------------
#   Created on Tue Oct 25 16:27:16 2011
#   copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_static_transforms = fawkescore fawkesutils fawkesaspects fawkesinterface \
			 fawkesblackboard fawkestf fawkesconfig
OBJS_static_transforms = static_transforms_plugin.o static_transforms_thread.o

OBJS_all    = $(OBJS_static_transforms)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  PLUGINS_all = $(PLUGINDIR)/static-transforms.so
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting static transforms plugin$(TNORMAL) (fawkestf not available)"
endif

include $(BUILDSYSDIR)/base.mk
