#*****************************************************************************
#              Makefile Build System for Fawkes: Gossip Library
#                            -------------------
#   Created on Tue Mar 04 10:54:41 2014
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

CFLAGS += $(CFLAGS_CPP11)

LIBS_libfawkesgossip = \
	fawkescore fawkesutils fawkesnetcomm fawkes_protobuf_comm
OBJS_libfawkesgossip = gossip_group.o gossip_group_manager.o

OBJS_all = $(OBJS_libfawkesgossip)

REQ_BOOST_LIBS = asio signals2 system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),11)
  CFLAGS  += $(CFLAGS_PROTOBUF) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  LIBS_all = $(LIBDIR)/libfawkesgossip.$(SOEXT)
else
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_protobuf $(WARN_TARGETS_BOOST)
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip library$(TNORMAL) (protobuf not found)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build gossip library$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
