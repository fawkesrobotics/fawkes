#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS Agent Plugin
#                            -------------------
#   Created on Sat Jun 16 15:00:02 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/clips.mk
include $(BUILDSYSDIR)/boost.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_clips_protobuf = \
	fawkescore fawkesutils fawkesaspects fawkesclipsaspect \
	fawkes_protobuf_comm fawkes_protobuf_clips
OBJS_clips_protobuf = clips-protobuf-plugin.o clips-protobuf-thread.o

OBJS_all = $(OBJS_clips_protobuf)

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_PROTOBUF)$(HAVE_CLIPS)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF)  $(CFLAGS_CLIPS) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) $(LDFLAGS_CLIPS) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  PLUGINS_all = $(PLUGINDIR)/clips-protobuf.so
else
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-protobuf plugin$(TNORMAL) (protobuf not found)"

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-protobuf plugin$(TNORMAL) (CLIPS not found)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_comm library$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
