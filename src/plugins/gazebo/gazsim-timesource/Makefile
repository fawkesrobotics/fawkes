#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-timesource 
#                            -------------------
#   Created on Sat Jun 16 14:42:34 2012
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

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk
# include $(BASEDIR)/src/plugins/gazebo/protobuf.mk


CFLAGS += $(CFLAGS_CPP11)

LIBS_gazsim_timesource = m fawkescore fawkesutils fawkesaspects fawkesgazeboaspect gazsim_msgs
OBJS_gazsim_timesource = gazsim_timesource_source.o gazsim_timesource_plugin.o gazsim_timesource_thread.o

OBJS_all = $(OBJS_gazsim_timesource)

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
	     $(CFLAGS_GAZEBO)
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
	     $(LDFLAGS_GAZEBO)

  PLUGINS_all = $(PLUGINDIR)/gazsim-timesource.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS += warning_boost
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_gazebo warning_protobuf warning_boost
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-timesource Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-timesource Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
warning_boost:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-timesource Plugin$(TNORMAL) " \
		"(required boost libs not installed)"

endif


include $(BUILDSYSDIR)/base.mk

