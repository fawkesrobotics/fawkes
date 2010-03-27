#*****************************************************************************
#            Makefile Build System for Fawkes : BlackBoard Tools
#                            -------------------
#   Created on Thu Oct 05 14:14:14 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_bb_cleanup = fawkescore fawkesutils fawkesconfig fawkesblackboard
OBJS_bb_cleanup = bb_cleanup.o

LIBS_bb_list = fawkescore fawkesutils fawkesconfig fawkesblackboard
OBJS_bb_list = bb_list.o

LIBS_bb_meminfo = fawkescore fawkesutils fawkesconfig fawkesblackboard
OBJS_bb_meminfo = bb_meminfo.o

OBJS_all = $(OBJS_bb_cleanup) $(OBJS_bb_list) $(OBJS_bb_meminfo)
BINS_all =	$(BINDIR)/bb_cleanup	\
		$(BINDIR)/bb_list	\
		$(BINDIR)/bb_meminfo

include $(BUILDSYSDIR)/base.mk

