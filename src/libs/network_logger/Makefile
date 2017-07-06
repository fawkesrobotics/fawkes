#*****************************************************************************
#           Makefile Build System for Fawkes: Network Logger
#                            -------------------
#   Created on Thu Jul 06 09:41:19 2017 (split out of logging lib)
#   Copyright (C) 2006-2017 by Tim Niemueller
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

LIBS_all  = $(LIBDIR)/libfawkesnetworklogger.so

LIBS_libfawkesnetworklogger = stdc++ pthread fawkescore fawkeslogging fawkesnetcomm
OBJS_libfawkesnetworklogger = network_logger.o
HDRS_libfawkesnetworklogger = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkesnetworklogger)

include $(BUILDSYSDIR)/base.mk

