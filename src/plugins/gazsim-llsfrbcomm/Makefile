#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-llsfrbcomm 
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/src/plugins/gazebo/protobuf.mk


CFLAGS += $(CFLAGS_CPP11)

LIBS_gazsim_llsfrbcomm = fawkescore fawkesutils fawkesaspects fawkes_protobuf_comm protobufmsgs fawkesgazeboaspect
OBJS_gazsim_llsfrbcomm = gazsim_llsfrbcomm_plugin.o gazsim_llsfrbcomm_thread.o

OBJS_all = $(OBJS_gazsim_llsfrbcomm)

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),11)
  CFLAGS  += $(CFLAGS_PROTOBUF) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
	     $(CFLAGS_GAZEBO)
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
	     $(LDFLAGS_GAZEBO)

  PLUGINS_all = $(PLUGINDIR)/gazsim-llsfrbcomm.so
else
  WARN_TARGETS += warning_protobuf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_protobuf
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting plugin$(TNORMAL) (protobuf not found)"
endif


include $(BUILDSYSDIR)/base.mk

