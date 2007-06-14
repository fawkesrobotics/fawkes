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
  WARN_TARGETS += warning_pkgconfig
endif


LIBXMLPP = $(shell $(PKGCONFIG) libxml++-2.6; echo $$?)
ifeq ($(LIBXMLPP),1)
  WARN_TARGETS += warning_libxmlpp
else
CFLAGS = -g \
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

ifneq ($(SRCDIR),)
all: $(WARN_TARGETS)
.PHONY: warning_pkgconfig
warning_pkgconfig:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting interface generator$(TNORMAL) (pkg-config not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Interfaces cannot be generated$(TNORMAL)"
.PHONY: warning_libxmlpp
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting interface generator$(TNORMAL) (libxml++[-devel] not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Interfaces cannot be generated$(TNORMAL)"
endif

OBJS_interfaces = $(notdir $(patsubst %.xml,%.o,$(wildcard $(SRCDIR)/*.xml)))
SRCS_interfaces = $(patsubst %.xml,%.cpp,$(wildcard $(SRCDIR)/*.xml))
HDRS_interfaces = $(patsubst %.xml,%.h,$(wildcard $(SRCDIR)/*.xml))

LIBS_libinterfaces = core interface
OBJS_libinterfaces = $(OBJS_interfaces)

OBJS_all = $(OBJS_libinterfaces) $(OBJS_interface_generator)
LIBS_all = $(LIBDIR)/libinterfaces.so

include $(BASEDIR)/etc/buildsys/config.mk


.SECONDARY: $(SRCS_interfaces) $(HDRS_interfaces) $(OBJS_all)

ifneq ($(OBJS_interface_generator),)
$(SRCS_interfaces): $(BINDIR)/interface_generator
$(SRCS_interfaces): %.cpp: %.xml
	$(SILENT) echo "$(INDENT_PRINT)--> Generating $(@F) (Interface XML Template)"
	$(SILENT)$(BINDIR)/interface_generator $<
endif

ifeq ($(MAKECMDGOALS),interface-clean)
.PHONY: interface-clean
interface-clean:
	$(SILENT) rm -f $(SRCS_interfaces) $(HDRS_interfaces)
endif

include $(BASEDIR)/etc/buildsys/base.mk

