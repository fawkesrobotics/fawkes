#*****************************************************************************
#              Makefile Build System for Fawkes : NetComm QA
#                            -------------------
#   Created on Fri Nov 10 10:24:17 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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

REQ_BOOST_LIBS = thread asio system signals2
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))
CFLAGS += $(CFLAGS_CPP11)

LIBS_qa_protobuf_comm_server = llsf_protobuf_comm llsf_msgs
OBJS_qa_protobuf_comm_server = qa_server.o

LIBS_qa_protobuf_comm_client = llsf_protobuf_comm llsf_msgs
OBJS_qa_protobuf_comm_client = qa_client.o

LIBS_qa_protobuf_comm_peer = llsf_protobuf_comm llsf_msgs
OBJS_qa_protobuf_comm_peer = qa_peer.o

OBJS_all = $(OBJS_qa_protobuf_comm_server) \
	   $(OBJS_qa_protobuf_comm_client) \
	   $(OBJS_qa_protobuf_comm_peer)

ifeq ($(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),11)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
  BINS_all = $(BINDIR)/qa_protobuf_comm_server \
	     $(BINDIR)/qa_protobuf_comm_client \
	     $(BINDIR)/qa_protobuf_comm_peer
endif

include $(BUILDSYSDIR)/base.mk
