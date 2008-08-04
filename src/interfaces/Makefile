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

ifneq ($(wildcard ../../etc/buildsys/config.mk),)
  # Fawkes
  BASEDIR = ../..
else
  # RCSoftX
  BASEDIR = ../../..
endif


include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/lua.mk
include $(SRCDIR)/generator/generator.mk

CFLAGS += -DINTERFACEDIR=\"$(INTERFACEDIR)\"

OBJS_fawkesinterfaces  = $(notdir $(patsubst %.xml,%.o,$(wildcard $(SRCDIR)/*.xml)))
SRCS_fawkesinterfaces  = $(patsubst %.xml,%.cpp,$(wildcard $(SRCDIR)/*.xml))
HDRS_fawkesinterfaces  = $(patsubst %.xml,%.h,$(wildcard $(SRCDIR)/*.xml))
TOLUA_fawkesinterfaces = $(patsubst %.xml,%.tolua,$(wildcard $(SRCDIR)/*.xml))

LIBS_libfawkesinterfaces = fawkescore fawkesinterface
OBJS_libfawkesinterfaces = $(OBJS_fawkesinterfaces)

CFLAGS_fawkesinterfaces_tolua = -Wno-unused-function
LIBS_lua_fawkesinterfaces = fawkescore fawkesinterface fawkesinterfaces $(TOLUA_LIBS)
OBJS_lua_fawkesinterfaces = fawkesinterfaces_tolua.o

OBJS_all = $(OBJS_libfawkesinterfaces) $(OBJS_interface_generator) \
	   $(OBJS_lua_fawkesinterfaces)

LIBS_all = $(LIBDIR)/libfawkesinterfaces.so
ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkesinterfaces.so
  TARGETS_all += $(SRCDIR)/fawkesinterfaces_tolua.cpp
else
  all: warning_tolua_wrapper
endif

include $(SRCDIR)/generator/generator_rules.mk

$(LUALIBDIR)/fawkesinterfaces.so: | $(LIBDIR)/libfawkesinterfaces.so

.SECONDARY: $(SRCS_fawkesinterfaces) $(HDRS_fawkesinterfaces) \
	$(OBJS_fawkesinterfaces) $(TOLUA_interfaces) $(OBJS_lua_interfaces)

ifeq ($(BUILD_INTERFACE_GENERATOR),1)
$(SRCS_fawkesinterfaces): $(BINDIR)/interface_generator
$(SRCS_fawkesinterfaces): %.cpp: %.xml
	$(SILENT) echo "$(INDENT_PRINT)--> Generating $(@F) (Interface XML Template)"
	$(SILENT)$(BINDIR)/interface_generator $<
endif

ifeq ($(MAKECMDGOALS),clean-interfaces)
.PHONY: clean-interfaces
clean-interfaces:
	$(SILENT) rm -f $(SRCS_fawkesinterfaces) $(HDRS_fawkesinterfaces)
endif

include $(BASEDIR)/etc/buildsys/base.mk

