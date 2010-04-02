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

ifneq ($(SRCDIR),)
include $(SRCDIR)/../netcomm.mk
else
include ../netcomm.mk
endif

CFLAGS  = $(CFLAGS_AVAHI)  -g
LDFALGS = $(LDFLAGS_AVAHI)

ifeq ($(HAVE_AVAHI),1)
LIBS_qa_netcomm_avahi_publisher = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_avahi_publisher = qa_avahi_publisher.o

LIBS_qa_netcomm_avahi_browser = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_avahi_browser = qa_avahi_browser.o

LIBS_qa_netcomm_avahi_resolver = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_avahi_resolver = qa_avahi_resolver.o

BINS_all +=	$(BINDIR)/qa_netcomm_avahi_publisher		\
		$(BINDIR)/qa_netcomm_avahi_browser		\
		$(BINDIR)/qa_netcomm_avahi_resolver
endif

LIBS_qa_netcomm_socket_typeof = fawkesnetcomm
OBJS_qa_netcomm_socket_typeof = qa_socket_typeof.o

LIBS_qa_netcomm_socket_stream = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_socket_stream = qa_socket_stream.o

LIBS_qa_netcomm_socket_datagram = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_socket_datagram = qa_socket_datagram.o

LIBS_qa_netcomm_socket_datagram_multicast = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_socket_datagram_multicast = qa_socket_datagram_multicast.o

LIBS_qa_netcomm_socket_datagram_broadcast = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_socket_datagram_broadcast = qa_socket_datagram_broadcast.o

LIBS_qa_netcomm_worldinfo = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_worldinfo = qa_worldinfo.o

LIBS_qa_netcomm_worldinfo_encryption = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_worldinfo_encryption = qa_worldinfo_encryption.o

LIBS_qa_netcomm_worldinfo_msgsizes = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_worldinfo_msgsizes = qa_worldinfo_msgsizes.o

LIBS_qa_netcomm_resolver = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_resolver = qa_resolver.o

LIBS_qa_netcomm_dynamic_buffer = fawkesnetcomm fawkesutils
OBJS_qa_netcomm_dynamic_buffer = qa_dynamic_buffer.o

OBJS_all =	$(OBJS_qa_netcomm_avahi_publisher)		\
		$(OBJS_qa_netcomm_avahi_browser)		\
		$(OBJS_qa_netcomm_avahi_resolver)		\
		$(OBJS_qa_netcomm_socket_typeof)		\
		$(OBJS_qa_netcomm_socket_stream)		\
		$(OBJS_qa_netcomm_socket_datagram)		\
		$(OBJS_qa_netcomm_socket_datagram_multicast)	\
		$(OBJS_qa_netcomm_socket_datagram_broadcast)	\
		$(OBJS_qa_netcomm_worldinfo)			\
		$(OBJS_qa_netcomm_worldinfo_encryption)		\
		$(OBJS_qa_netcomm_worldinfo_msgsizes)		\
		$(OBJS_qa_netcomm_resolver)			\
		$(OBJS_qa_netcomm_dynamic_buffer)

BINS_all +=	$(BINDIR)/qa_netcomm_socket_typeof		\
		$(BINDIR)/qa_netcomm_socket_stream		\
		$(BINDIR)/qa_netcomm_socket_datagram		\
		$(BINDIR)/qa_netcomm_socket_datagram_multicast	\
		$(BINDIR)/qa_netcomm_socket_datagram_broadcast	\
		$(BINDIR)/qa_netcomm_worldinfo			\
		$(BINDIR)/qa_netcomm_worldinfo_encryption	\
		$(BINDIR)/qa_netcomm_worldinfo_msgsizes		\
		$(BINDIR)/qa_netcomm_resolver			\
		$(BINDIR)/qa_netcomm_dynamic_buffer

include $(BUILDSYSDIR)/base.mk

