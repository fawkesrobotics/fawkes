#*****************************************************************************
#            Makefile Build System for Fawkes: tf example plugin
#                            -------------------
#   Created on Tue Oct 25 17:57:57 2011
#   copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_tf_example = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkestf
OBJS_tf_example = tf_example_plugin.o tf_example_thread.o

OBJS_all    = $(OBJS_tf_example)


ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  PLUGINS_all = $(PLUGINDIR)/tf_example.so
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tf example plugin$(TNORMAL) (fawkestf not available)"
endif

include $(BUILDSYSDIR)/base.mk
