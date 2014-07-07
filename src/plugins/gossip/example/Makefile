#*****************************************************************************
#          Makefile Build System for Fawkes: Gossip Communication Example
#                            -------------------
#   Created on Wed Mar 05 14:24:23 2014
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk

LIBS_gex_sender = \
	fawkescore fawkesutils fawkesaspects fawkes_protobuf_comm \
	fawkesgossip fawkesgossipaspect gossip_example_msgs
OBJS_gex_sender = gex_sender_plugin.o gex_sender_thread.o

LIBS_gex_receiver = \
	fawkescore fawkesutils fawkesaspects fawkes_protobuf_comm \
	fawkesgossip fawkesgossipaspect gossip_example_msgs
OBJS_gex_receiver = gex_receiver_plugin.o gex_receiver_thread.o

OBJS_all = $(OBJS_gex_sender) $(OBJS_gex_receiver)

REQ_BOOST_LIBS = system signals2
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_CPP11)$(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(CFLAGS_CPP11) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  PLUGINS_all = $(PLUGINDIR)/gex-sender.$(SOEXT) $(PLUGINDIR)/gex-receiver.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

# Protobuf messages
PROTOBUF_all = gossip_example_msgs
MSGS_gossip_example_msgs = $(notdir $(patsubst %.proto,%,$(wildcard $(SRCDIR)/*.proto)))


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_cpp11 warning_protobuf $(WARN_TARGETS_BOOST)
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip examples$(TNORMAL) (C++11 not supported)"

warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip examples$(TNORMAL) (protobuf not found)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip examples$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/protobuf_msgs.mk
include $(BUILDSYSDIR)/base.mk
