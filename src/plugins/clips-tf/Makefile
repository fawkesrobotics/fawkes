#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS Transform Plugin
#                            -------------------
#   Created on Sat Apr 11 18:06:17 2015
#   Copyright (C) 2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/clips.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_clips_tf = fawkescore fawkesutils fawkesaspects \
		fawkesclipsaspect fawkestf
OBJS_clips_tf = clips_tf_plugin.o clips_tf_thread.o

OBJS_all = $(OBJS_clips_tf)

ifeq ($(HAVE_CPP11)$(HAVE_CLIPS)$(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_CLIPS)  $(CFLAGS_TF) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_CLIPS) $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/clips-tf.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_clips warning_cpp11
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-tf plugin$(TNORMAL) ($(CLIPS_ERROR))"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-tf plugin$(TNORMAL) (fawkestf not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-tf plugin$(TNORMAL) (C++11 not supported)"

endif

include $(BUILDSYSDIR)/base.mk
