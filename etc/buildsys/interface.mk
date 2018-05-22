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

ifndef __buildsys_config_mk_
$(error config.mk must be included before interface.mk)
endif

ifndef __buildsys_interface_mk_
__buildsys_interface_mk_ := 1

include $(BUILDSYSDIR)/lua.mk
include $(BUILDSYSDIR)/libxmlpp.mk

ifneq ($(PKGCONFIG),)
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

ifneq ($(INTERFACES_all),)
  $(foreach I,$(INTERFACES_all),						\
	$(eval LIBS_interfaces_lib$I        = $$(_LIBS_INTERFACE))		\
	$(eval LDFLAGS_interfaces_lib$I     = )					\
	$(eval OBJS_interfaces_lib$I        = $I.o)                   \
	$(eval HDRS_interfaces_lib$I        = $I.h)                   \
	$(eval INST_LIB_SUBDIR_interfaces_lib$I   = $(FFLIBSUBDIR))		\
	$(eval INST_HDRS_SUBDIR_interfaces_lib$I  = interfaces)			\
	$(eval OBJS_all                    += $$(OBJS_interfaces_lib$I))	\
	$(eval INTERFACES_SRCS             += $(SRCDIR)/$I.cpp)			\
	$(eval INTERFACES_TOLUA            += $(SRCDIR)/$I.tolua)		\
	$(eval CLEAN_FILES                 += $(SRCDIR)/$I.cpp $(SRCDIR)/$I.tolua $(IFACESRCDIR)/$I.h) \
	$(eval INTERFACES_HDRS             += $(IFACESRCDIR)/$I.h)		\
	$(eval INTERFACES_LIBS             += $(IFACEDIR)/lib$I.so)		\
	$(eval INTERFACES_TOUCH            += $(SRCDIR)/$(OBJDIR)/$I.touch)	\
	$(eval INTERFACES_OBJS             += $I.o)				\
	$(eval LIBS_all                    += $$(IFACEDIR)/lib$I.so)		\
										\
	$(eval TOLUA_ALL                   += $(I).tolua)			\
	$(eval TOLUA_SRCS                  += $(I)_tolua.cpp)			\
	$(eval LIBS_lua_interfaces_$(I)     = $$(_LIBS_TOLUA) $I)		\
	$(eval OBJS_lua_interfaces_$(I)     = $(I)_tolua.o)			\
	$(eval CFLAGS_$(I)_tolua	    = $$(_CFLAGS_TOLUA))		\
	$(eval LDFLAGS_lua_interfaces_$I    = $$(_LDFLAGS_TOLUA))		\
	$(eval TOLUA_$(I)                   = $(I).tolua)			\
	$(eval TOLUA_PKGPREFIX_$(I)         = interfaces_)			\
	$(eval OBJS_all                    += $(I)_tolua.o)			\
	$(eval LIBS_all_tolua              += $$(LUALIBDIR)/interfaces/$(I).so)	\
  )

  ifeq ($(IFACESRCDIR),$(SRCDIR))
    INTERFACE_GENERATOR_BUILT = 1
  else
    ifneq ($(wildcard $(BINDIR)/ffifacegen),)
      INTERFACE_GENERATOR_BUILT = 1
    endif
  endif

ifneq ($(HAVE_INTERFACE_GENERATOR)$(INTERFACE_GENERATOR_BUILT),11)
  ERROR_TARGETS += error_ifacegen
endif

ifeq ($(OBJSSUBMAKE)$(HAVE_INTERFACE_GENERATOR)$(INTERFACE_GENERATOR_BUILT),111)

$(INTERFACES_SRCS): $(SRCDIR)/%.cpp: $(SRCDIR)/$(OBJDIR)/%.touch
$(INTERFACES_HDRS): $(IFACESRCDIR)/%.h: $(SRCDIR)/$(OBJDIR)/%.touch
$(INTERFACES_TOLUA): $(SRCDIR)/%.tolua: $(SRCDIR)/$(OBJDIR)/%.touch

$(INTERFACES_TOUCH): $(SRCDIR)/$(OBJDIR)/%.touch: $(SRCDIR)/%.xml
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[IFC] $(PARENTDIR)$(TBOLDGRAY)$*.cpp$(TNORMAL)"
	$(SILENT)mkdir -p $(SRCDIR)
	$(SILENT)$(BINDIR)/ffifacegen -d $(SRCDIR) $<
	$(SILENT)if [ "$(SRCDIR)" != "$(IFACESRCDIR)" ]; then \
		mv -f $(SRCDIR)/$*.h $(IFACESRCDIR)/$*.h; \
	fi
	$(SILENT) mkdir -p $(@D)
	$(SILENT) touch $@

.SECONDARY: $(INTERFACES_SRCS) $(INTERFACES_HDRS) $(TOLUA_SRCS) $(TOLUA_ALL)

.PHONY: error_ifacegen
error_ifacegen:
	$(SILENT)echo -e "$(INDENT_PRINT)[ERROR] $(TRED)Fawkes interface generator not available$(TNORMAL)"
	$(SILENT)exit 1

endif # OBJSSUBMAKE != 1

ifneq ($(PLUGINS_all),)
$(PLUGINS_all:%.so=%.$(SOEXT)): | $(INTERFACES_LIBS:%.so=%.$(SOEXT))
endif
ifneq ($(filter-out $(BINDIR)/ffifacegen,$(BINS_all)),)
$(BINS_all): | $(INTERFACES_LIBS)
endif

ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LIBS_all_tolua)

$(LIBS_all_tolua): $(LUALIBDIR)/interfaces/%.$(SOEXT): | $(IFACEDIR)/lib%.$(SOEXT)

else
all: warning_tolua_wrapper
endif

endif # INTERFACES_all != ""
endif # __buildsys_interface_mk_
