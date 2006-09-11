#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
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

BASEDIR = ../..

OBJS_libutils =	\
		plugin/plugin_loader.o			\
		system/dynamic_module/module_dl.o			\
		system/dynamic_module/module_manager_factory.o		\
		system/file.o
LIBS_libutils = stdc++ dl

OBJS_all = $(OBJS_libutils)
LIBS_all  = $(LIBDIR)/libutils.so

include $(BASEDIR)/etc/buildsys/base.mk

