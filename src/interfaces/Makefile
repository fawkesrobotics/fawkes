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

INTERFACES = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))

LIBS_interface = fawkescore fawkesinterface

LIBS_tolua    = fawkescore fawkesinterface $(TOLUA_LIBS)
CFLAGS_tolua  = -Wno-unused-function $(CFLAGS_LUA)
LDFLAGS_tolua = $(LDFLAGS_LUA)

$(foreach I,$(INTERFACES),							\
	$(eval LIBS_interfaces_$(I)      = $$(LIBS_interface))			\
	$(eval OBJS_interfaces_$(I)      = $(I).o)				\
	$(eval OBJS_all                 += $$(OBJS_interfaces_$(I)))		\
	$(eval SRCS_all                 += $(SRCDIR)/$(I).cpp)			\
	$(eval HDRS_all                 += $(I).h)				\
	$(eval LIBS_all                 += $$(LIBDIR)/interfaces/$(I).so)	\
	$(eval LIBS_all_interfaces      += $$(LIBDIR)/interfaces/$(I).so)	\
										\
	$(eval TOLUA_all                += $(I).tolua)				\
	$(eval TOLUA_srcs               += $(I)_tolua.cpp)			\
	$(eval LIBS_lua_interfaces_$(I)  = $$(LIBS_tolua) $I)			\
	$(eval OBJS_lua_interfaces_$(I)  = $(I)_tolua.o)			\
	$(eval CFLAGS_$(I)_tolua	 = $$(CFLAGS_tolua))			\
	$(eval LDFLAGS_lua_interfaces_$I = $$(LDFLAGS_tolua))			\
	$(eval TOLUA_$(I)                = $(I).tolua)				\
	$(eval TOLUA_PKGPREFIX_$(I)      = interfaces_)				\
	$(eval OBJS_all                 += $(I)_tolua.o)			\
	$(eval LIBS_all_tolua           += $$(LUALIBDIR)/interfaces/$(I).so)	\
)

# Compatibility library that links to the standard Fawkes interfaces
LIBS_libfawkesinterfaces = $(INTERFACES)
LIBS_all += $(LIBDIR)/libfawkesinterfaces.so

ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LIBS_all_tolua)
else
  all: warning_tolua_wrapper
endif

# Dependency and secondary targets
$(LIBDIR)/libfawkesinterfaces.so: | $(LIBS_all_interfaces)
$(LUALIBDIR)/interfaces/%.so: | $(LIBDIR)/interfaces/%.so
.SECONDARY: $(SRCS_all) $(HDRS_all) $(TOLUA_srcs) $(TOLUA_all)


# Interface generator stuff
include $(SRCDIR)/generator/generator_rules.mk

ifeq ($(BUILD_INTERFACE_GENERATOR),1)
$(SRCS_all): $(BINDIR)/interface_generator
$(SRCS_all): %.cpp: %.xml
	$(SILENT) echo "$(INDENT_PRINT)--> Generating $(@F) (Interface XML Template)"
	$(SILENT)$(BINDIR)/interface_generator -d $(SRCDIR) $<
endif

ifeq ($(MAKECMDGOALS),clean-interfaces)
.PHONY: clean-interfaces
clean-interfaces:
	$(SILENT) rm -f $(SRCS_fawkesinterfaces) $(HDRS_fawkesinterfaces)
endif

include $(BASEDIR)/etc/buildsys/base.mk

