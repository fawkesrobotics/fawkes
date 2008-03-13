#*****************************************************************************
#                Makefile Build System for Fawkes : Interfaces lib
#                            -------------------
#   Created on Tue Oct 10 17:37:18 2006
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

BASEDIR = ../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/lua.mk
include $(SRCDIR)/generator/generator.mk

CFLAGS += -DINTERFACEDIR=\"$(realpath $(BASEDIR)/src/interfaces)\"

OBJS_interfaces  = $(notdir $(patsubst %.xml,%.o,$(wildcard $(SRCDIR)/*.xml)))
SRCS_interfaces  = $(patsubst %.xml,%.cpp,$(wildcard $(SRCDIR)/*.xml))
HDRS_interfaces  = $(patsubst %.xml,%.h,$(wildcard $(SRCDIR)/*.xml))
TOLUA_interfaces = $(patsubst %.xml,%.tolua,$(wildcard $(SRCDIR)/*.xml))

LIBS_libinterfaces = core interface
OBJS_libinterfaces = $(OBJS_interfaces)

CFLAGS_interfaces_tolua = -Wno-unused-function
LIBS_lua_interfaces = core interface interfaces tolua++
OBJS_lua_interfaces = interfaces_tolua.o

OBJS_all = $(OBJS_libinterfaces) $(OBJS_interface_generator) \
	   $(OBJS_lua_interfaces)

LIBS_all = $(LIBDIR)/libinterfaces.so
ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LIBDIR)/lua/interfaces.so
  TARGETS_all += $(SRCDIR)/interfaces_tolua.cpp
else
  all: warning_tolua_wrapper
endif

include $(SRCDIR)/generator/generator_rules.mk

$(LIBDIR)/lua/interfaces.so: $(LIBDIR)/libinterfaces.so

.SECONDARY: $(SRCS_interfaces) $(HDRS_interfaces) $(OBJS_interfaces) \
	$(TOLUA_interfaces) $(SRCDIR)/interfaces_tolua.cpp \
	$(OBJS_lua_interfaces)

ifeq ($(BUILD_INTERFACE_GENERATOR),1)
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

