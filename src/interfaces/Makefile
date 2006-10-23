#*****************************************************************************
#                Makefile Build System for Fawkes : Interfaces lib
#                            -------------------
#   Created on Tue Oct 10 17:37:18 2006
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

ifeq ($(PKGCONFIG),)
$(warning No pkg-config installed, you cannot build the interface generator! Interfaces cannot be generated.)
else

LIBXMLPP = $(shell $(PKGCONFIG) libxml++-2.6; echo $$?)
ifeq ($(LIBXMLPP),1)
$(warning libxml++-devel not installed on your system, you cannot build the interface generator! Interfaces cannot be generated.)
else
CFLAGS = -g -DLIBDIR=\"$(LIBDIR)\" \
	-DINTERFACEDIR=\"$(realpath $(SRCDIR))\" \
	$(shell pkg-config --cflags libxml++-2.6)

LIBS_interface_generator = core utils
LDFLAGS_interface_generator = $(shell pkg-config --libs libxml++-2.6)
OBJS_interface_generator = generator/constant.o		\
			   generator/enum_constant.o	\
			   generator/field.o		\
			   generator/generator.o	\
			   generator/main.o		\
			   generator/message.o		\
			   generator/parser.o		\
			   generator/type_checker.o

BINS_all = $(BINDIR)/interface_generator
endif
endif

OBJS_interfaces = $(notdir $(patsubst %.xml,%.o,$(wildcard $(SRCDIR)/*.xml)))
SRCS_interfaces = $(patsubst %.xml,%.cpp,$(wildcard $(SRCDIR)/*.xml))
HDRS_interfaces = $(patsubst %.xml,%.h,$(wildcard $(SRCDIR)/*.xml))

LIBS_libinterfaces = core
OBJS_libinterfaces = interface.o message.o message_queue.o $(OBJS_interfaces)

OBJS_all = $(OBJS_libinterfaces) $(OBJS_interface_generator)
LIBS_all = $(LIBDIR)/libinterfaces.so

include $(BASEDIR)/etc/buildsys/config.mk


.SECONDARY: $(SRCS_interfaces)
$(SRCS_interfaces): $(BINDIR)/interface_generator
$(SRCS_interfaces): %.cpp: %.xml
	$(SILENT) echo "$(INDENT_PRINT)--> Generating $(@F) (Interface XML Template)"
	$(SILENT)$(BINDIR)/interface_generator $<

.PHONY: interface-clean
interface-clean:
	$(SILENT) rm -f $(SRCS_interfaces) $(HDRS_interfaces)

include $(BASEDIR)/etc/buildsys/base.mk

