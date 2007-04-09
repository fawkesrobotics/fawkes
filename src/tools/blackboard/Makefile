#*****************************************************************************
#              Makefile Build System for Fawkes : BlackBoard QA
#                            -------------------
#   Created on Thu Oct 05 14:14:14 2006
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
#           $Id: Makefile 31 2006-10-23 17:41:08Z tim $
# last modified: $Date: 2006-10-23 19:41:08 +0200 (Mo, 23 Okt 2006) $
#            by: $Author: tim $
#
#*****************************************************************************

BASEDIR = ../../..

CFLAGS = -g -DLIBDIR=\"$(LIBDIR)\"

LIBS_bb_cleanup = core utils blackboard
OBJS_bb_cleanup = bb_cleanup.o

LIBS_bb_list = core utils blackboard
OBJS_bb_list = bb_list.o

LIBS_bb_meminfo = core utils blackboard
OBJS_bb_meminfo = bb_meminfo.o

OBJS_all = $(OBJS_bb_cleanup) $(OBJS_bb_list) $(OBJS_bb_meminfo)
BINS_all =	$(BINDIR)/bb_cleanup	\
		$(BINDIR)/bb_list	\
		$(BINDIR)/bb_meminfo

include $(BASEDIR)/etc/buildsys/base.mk

