#*****************************************************************************
#     Makefile Build System for Fawkes: AMCL set pose tool
#                            -------------------
#   Created on Mon Mar 23 14:26:11 2015
#   copyright (C) 2015 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_ffset_pose = fawkescore fawkesutils fawkesinterface \
	    fawkesblackboard fawkesnetcomm fawkesconfig \
	    LocalizationInterface
OBJS_ffset_pose = ffset_pose.o
OBJS_all    = $(OBJS_ffset_pose)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)

  BINS_all     = $(BINDIR)/ffset_pose
  MANPAGES_all = $(MANDIR)/man1/ffset_pose.1
else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting amcl plugin and libs$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
