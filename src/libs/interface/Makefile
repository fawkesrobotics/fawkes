#*****************************************************************************
#               Makefile Build System for Fawkes : Interface lib
#                            -------------------
#   Created on Fri Jan 12 18:08:27 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk

LIBS_libinterface = core
OBJS_libinterface = interface.o interface_info.o message.o message_queue.o

OBJS_all = $(OBJS_libinterface)
LIBS_all = $(LIBDIR)/libinterface.so

include $(BASEDIR)/etc/buildsys/config.mk


.SECONDARY: $(OBJS_all)

include $(BASEDIR)/etc/buildsys/base.mk

