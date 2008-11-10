#*****************************************************************************
#              Makefile Build System for Fawkes : Interface related
#                            -------------------
#   Created on Fri Nov 07 16:50:34 2008
#   copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_interface_mk_
__buildsys_interface_mk_ := 1

include $(BASEDIR)/etc/buildsys/lua.mk
include $(BASEDIR)/etc/buildsys/ext/gmsl

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLPP   = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
  CFLAGS_LIBXMLPP = $(shell $(PKGCONFIG) --cflags libxml++-2.6)
  HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --exists 'libcrypto'; echo $${?/1/}),1,0)
  LIBCRYPTO_PKG  := libcrypto
  CFLAGS_LIBCRYPTO  = $(shell $(PKGCONFIG) --cflags 'libcrypto')
  LDFLAGS_LIBCRYPTO = $(shell $(PKGCONFIG) --libs 'libcrypto')
  ifneq ($(HAVE_LIBCRYPTO),1)
    HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --exists 'openssl'; echo $${?/1/}),1,0)
    LIBCRYPTO_PKG  := openssl
    CFLAGS_LIBCRYPTO  = $(shell $(PKGCONFIG) --cflags 'openssl')
    LDFLAGS_LIBCRYPTO = $(shell $(PKGCONFIG) --libs 'openssl')
  endif
endif

ifeq ($(HAVE_LIBXMLPP)$(HAVE_LIBCRYPTO),11)
  HAVE_INTERFACE_GENERATOR=1
else
  ifneq ($(HAVE_LIBXMLPP),1)
    INTERFACE_GENERATOR_MISSING_PACKAGE += "libxml++"
  endif
  ifneq ($(HAVE_LIBCRYPTO),1)
    INTERFACE_GENERATOR_MISSING_PACKAGE += "libcrypto/openssl"
  endif

  INTERFACE_GENERATOR_ERROR = ($(call merge, and ,$(INTERFACE_GENERATOR_MISSING_PACKAGE)) not installed)
endif

_LIBS_INTERFACE   = fawkescore fawkesinterface
_LIBS_TOLUA       = fawkescore fawkesinterface $(TOLUA_LIBS)
_CFLAGS_TOLUA     = -Wno-unused-function $(CFLAGS_LUA)
_LDFLAGS_TOLUA    = $(LDFLAGS_LUA)

$(foreach I,$(INTERFACES_all),							\
	$(eval LIBS_interfaces_$(I)      = $$(_LIBS_INTERFACE))			\
	$(eval OBJS_interfaces_$(I)      = $(I).o)				\
	$(eval OBJS_all                 += $$(OBJS_interfaces_$(I)))		\
	$(eval INTERFACES_SRCS          += $(SRCDIR)/$(I).cpp)			\
	$(eval INTERFACES_HDRS          += $(I).h)				\
	$(eval LIBS_all                 += $$(LIBDIR)/interfaces/$(I).so)	\
										\
	$(eval TOLUA_ALL                += $(I).tolua)				\
	$(eval TOLUA_SRCS               += $(I)_tolua.cpp)			\
	$(eval LIBS_lua_interfaces_$(I)  = $$(_LIBS_TOLUA) $I)			\
	$(eval OBJS_lua_interfaces_$(I)  = $(I)_tolua.o)			\
	$(eval CFLAGS_$(I)_tolua	 = $$(_CFLAGS_TOLUA))			\
	$(eval LDFLAGS_lua_interfaces_$I = $$(_LDFLAGS_TOLUA))			\
	$(eval TOLUA_$(I)                = $(I).tolua)				\
	$(eval TOLUA_PKGPREFIX_$(I)      = interfaces_)				\
	$(eval OBJS_all                 += $(I)_tolua.o)			\
	$(eval LIBS_all_tolua           += $$(LUALIBDIR)/interfaces/$(I).so)	\
)

ifeq ($(IFACEDIR),$(SRCDIR))
  INTERFACE_GENERATOR_BUILD = 1
else
  ifneq ($(wildcard $(BINDIR)/interface_generator),)
    INTERFACE_GENERATOR_BUILD = 1
  endif
endif

ifeq ($(HAVE_INTERFACE_GENERATOR)$(INTERFACE_GENERATOR_BUILD),11)
  ifneq ($(INTERFACES_all),)
    $(INTERFACES_SRCS): $(BINDIR)/interface_generator

    $(INTERFACES_SRCS): %.cpp: %.xml
	$(SILENT) echo "$(INDENT_PRINT)--> Generating $(@F) (Interface XML Template)"
	$(SILENT)$(BINDIR)/interface_generator -d $(SRCDIR) $<
	$(if $(filter-out $(IFACEDIR),$(SRCDIR)),$(SILENT)cp -a $*.h $(IFACEDIR))
  endif

  ifeq ($(MAKECMDGOALS),clean-interfaces)
  .PHONY: clean-interfaces
  clean-interfaces:
	$(SILENT) rm -f $(addsuffix .cpp,$(INTERFACES_all)) \
			$(addsuffix .h,$(INTERFACES_all))
  endif
else
  ifneq ($(IFACEDIR),$(SRCDIR))
    $(INTERFACES_SRCS): %.cpp: %.xml
	$(SILENT)if [ ! -e $*.h -o ! -e $*.cpp ]; then \
		echo -e "$(INDENT_PRINT)--- $(TRED) Interfaces cannot be generated and pre-generated code does not exist!$(TNORMAL)"; \
		exit 1; \
	else \
		echo -e "$(INDENT_PRINT)--- $(TYELLOW)Generator not avaible, only copying $(@F)$(TNORMAL)"; \
		cp -a $*.h $(IFACEDIR); \
		touch $*.cpp; \
	fi
  endif
endif

ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LIBS_all_tolua)

  $(LUALIBDIR)/interfaces/%.so: | $(LIBDIR)/interfaces/%.so

else
  all: warning_tolua_wrapper
endif


.SECONDARY: $(INTERFACES_SRCS) $(INTERFACES_HDRS) $(TOLUA_SRCS) $(TOLUA_ALL)

endif # __buildsys_interface_mk_
