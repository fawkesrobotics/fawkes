#*****************************************************************************
#              Makefile Build System for Fawkes: Gossip Plugin
#                            -------------------
#   Created on Fri Feb 28 11:06:42 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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

PRESUBDIRS = gossip aspect

CFLAGS += $(CFLAGS_CPP11)

LIBS_gossip = \
	fawkescore fawkesutils fawkesaspects fawkes_protobuf_comm \
	fawkesgossip fawkesgossipaspect
OBJS_gossip = gossip_plugin.o gossip_thread.o

OBJS_all = $(OBJS_gossip)

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),11)
  CFLAGS  += $(CFLAGS_PROTOBUF) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  PLUGINS_all = $(PLUGINDIR)/gossip.so
else
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif


aspect: gossip


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_protobuf $(WARN_TARGETS_BOOST)
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip plugin$(TNORMAL) (protobuf not found)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
