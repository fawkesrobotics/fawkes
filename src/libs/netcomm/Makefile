#*****************************************************************************
#               Makefile Build System for Fawkes: NetComm Library
#                            -------------------
#   Created on Tue Nov 07 16:43:45 2006
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
ifeq ($(PKGCONFIG),)
$(error No pkg-config installed, you cannot build the NetComm library!)
else
  AVAHI = $(shell $(PKGCONFIG) avahi-client; echo $$?)
  ifeq ($(AVAHI),1)
    $(error avahi-devel not installed on your system, you cannot build the NetComm library!)
  endif
endif

CFLAGS = -g $(shell $(PKGCONFIG) --cflags avahi-client)

LIBS_libnetcomm = core
LDFLAGS_libnetcomm = $(shell pkg-config --libs avahi-client)
OBJS_libnetcomm =					\
		  dns-sd/avahi_browser.o		\
		  dns-sd/avahi_service.o		\
		  dns-sd/avahi_service_publisher.o	\
		  dns-sd/avahi_thread.o			\
		  fawkes/acceptor_thread.o		\
		  fawkes/client.o			\
		  fawkes/client_thread.o		\
		  fawkes/handler.o			\
		  fawkes/message.o			\
		  fawkes/message_queue.o		\
		  fawkes/network_thread.o		\
		  fawkes/transceiver.o			\
		  socket/datagram.o			\
		  socket/socket.o			\
		  socket/stream.o

OBJS_all = $(OBJS_libnetcomm)
LIBS_all  = $(LIBDIR)/libnetcomm.so

include $(BASEDIR)/etc/buildsys/base.mk

