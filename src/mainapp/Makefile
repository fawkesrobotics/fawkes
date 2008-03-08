#*****************************************************************************
#            Makefile Build System for Fawkes : Main Application
#                            -------------------
#   Created on Thu Nov 02 19:07:47 2006
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

BASEDIR = ../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/src/libs/netcomm/netcomm.mk

# Add to enable TimeTracker support in Fawkes main application
#CFLAGS += -DUSE_TIMETRACKER

# We need Avahi CFLAGS because of includes in Avahi source, on BSD this is
# in /usr/local/include and thus it needs extra flags
ifeq ($(HAVE_AVAHI),1)
  CFLAGS += $(CFLAGS_AVAHI)
endif

LIBS_libfawkesmainappmsgs = core utils netcomm
OBJS_libfawkesmainappmsgs = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*_message.cpp)))))

LIBS_fawkes = core utils interfaces blackboard netcomm config aspects fawkesmainappmsgs
OBJS_fawkes = $(filter-out $(OBJS_fawkesmainappmsgs),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))

OBJS_all =	$(OBJS_fawkes)			\
		$(OBJS_fawkesmainappmsgs)
LIBS_all =	$(LIBDIR)/libfawkesmainappmsgs.so
BINS_all =	$(BINDIR)/fawkes

include $(BASEDIR)/etc/buildsys/base.mk

