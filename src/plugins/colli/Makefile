
#*****************************************************************************
#              Makefile Build System for Fawkes: Colli Plugin
#                            -------------------
#   Created on Sat Jul 13 12:00:00 2013
#   Copyright (C) 2013 by AllemaniACs
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

LIBS_colli = fawkescore fawkesutils fawkesaspects fawkestf \
             fawkesinterface MotorInterface Laser360Interface  NavigatorInterface
OBJS_colli = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
else
  WARN_TARGETS += warning_tf
endif

PLUGINS_all = $(PLUGINDIR)/colli.$(SOEXT)
OBJS_all    = $(OBJS_colli)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_ros warning_nav_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tf support for colli$(TNORMAL) (fawkestf not available)"
endif

include $(BUILDSYSDIR)/base.mk
