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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../../..

CFLAGS = -g

LIBS_qa_netcomm_avahi_publisher = netcomm utils
OBJS_qa_netcomm_avahi_publisher = qa_avahi_publisher.o

LIBS_qa_netcomm_avahi_browser = netcomm utils
OBJS_qa_netcomm_avahi_browser = qa_avahi_browser.o

LIBS_qa_netcomm_socket_typeof = netcomm
OBJS_qa_netcomm_socket_typeof = qa_socket_typeof.o

LIBS_qa_netcomm_socket_stream = netcomm utils
OBJS_qa_netcomm_socket_stream = qa_socket_stream.o

LIBS_qa_netcomm_socket_seqpacket = netcomm utils
OBJS_qa_netcomm_socket_seqpacket = qa_socket_seqpacket.o

LIBS_qa_netcomm_socket_datagram = netcomm utils
OBJS_qa_netcomm_socket_datagram = qa_socket_datagram.o

BINS_all =	$(BINDIR)/qa_netcomm_avahi_publisher	\
		$(BINDIR)/qa_netcomm_avahi_browser	\
		$(BINDIR)/qa_netcomm_socket_typeof	\
		$(BINDIR)/qa_netcomm_socket_stream	\
		$(BINDIR)/qa_netcomm_socket_seqpacket	\
		$(BINDIR)/qa_netcomm_socket_datagram

include $(BASEDIR)/etc/buildsys/base.mk

