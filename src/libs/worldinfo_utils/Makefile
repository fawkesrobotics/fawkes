#*****************************************************************************
#        Makefile Build System for Fawkes: WorldInfo Utilitiy Library
#                            -------------------
#   Created on Mon April 14 23:04:38 2008
#   copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
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

LIBS_libfawkesworldinfo_utils = stdc++ fawkesnetcomm fawkesgeometry fawkesutils
OBJS_libfawkesworldinfo_utils = data_container.o
HDRS_libfawkesworldinfo_utils = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkesworldinfo_utils)
LIBS_all = $(LIBDIR)/libfawkesworldinfo_utils.so

include $(BUILDSYSDIR)/base.mk
