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

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLPP    = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
  CFLAGS_LIBXMLPP  = $(shell $(PKGCONFIG) --cflags libxml++-2.6)
  LDFLAGS_LIBXMLPP = $(shell $(PKGCONFIG) --libs libxml++-2.6)
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
	$(eval OBJS_interfaces_lib$I        = $I.o)				\
	$(if $(subst $(abspath $(IFACESRCDIR)),,$(abspath $(SRCDIR))),		\
		$(eval HDRS_interfaces_lib$I = $I.h_ext)			\
		$(eval HDR_RENAME_$I.h_ext   = $I.h)				\
	, 									\
		$(eval HDRS_interfaces_lib$I = $I.h)				\
	) 									\
	$(eval INST_LIB_SUBDIR_interfaces_lib$I   = $(FFLIBSUBDIR))		\
	$(eval INST_HDRS_SUBDIR_interfaces_lib$I  = interfaces)			\
	$(eval OBJS_all                    += $$(OBJS_interfaces_lib$I))	\
	$(eval INTERFACES_SRCS             += $(SRCDIR)/$I.cpp)			\
	$(eval INTERFACES_TOLUA            += $(SRCDIR)/$I.tolua)		\
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
    INTERFACE_GENERATOR_BUILD = 1
  else
    ifneq ($(wildcard $(BINDIR)/ffifacegen),)
      INTERFACE_GENERATOR_BUILD = 1
    endif
  endif

ifeq ($(OBJSSUBMAKE),1)

$(INTERFACES_SRCS): $(SRCDIR)/%.cpp: $(SRCDIR)/$(OBJDIR)/%.touch
$(INTERFACES_HDRS): $(IFACESRCDIR)/%.h: $(SRCDIR)/$(OBJDIR)/%.touch
$(INTERFACES_TOLUA): $(SRCDIR)/%.tolua: $(SRCDIR)/$(OBJDIR)/%.touch

$(INTERFACES_TOUCH): $(SRCDIR)/$(OBJDIR)/%.touch: $(SRCDIR)/%.xml
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[IFC] $(PARENTDIR)$(TBOLDGRAY)$*.cpp$(TNORMAL)"
  ifeq ($(HAVE_INTERFACE_GENERATOR)$(INTERFACE_GENERATOR_BUILD),11)
	$(SILENT)$(BINDIR)/ffifacegen -d $(SRCDIR) $<
	$(SILENT)mkdir -p $(IFACESRCDIR)
	$(if $(filter-out $(IFACESRCDIR),$(SRCDIR)),$(SILENT)mv $(SRCDIR)/$*.h $(SRCDIR)/$*.h_ext; cp -a $(SRCDIR)/$*.h_ext $(IFACESRCDIR)/$*.h)
  else
    ifneq ($(abspath $(IFACESRCDIR)),$(abspath $(SRCDIR)))
	$(SILENT) if [ ! -e $(SRCDIR)/$*.h_ext -o ! -e $(SRCDIR)/$*.cpp ]; then \
		echo -e "$(INDENT_PRINT)[ERR] $(TRED)Interfaces cannot be generated and pre-generated code does not exist!$(TNORMAL)"; \
		exit 1; \
	else \
		echo -e "$(INDENT_PRINT)[WARN] $(TYELLOW)Generator not available, only copying $*.h(_ext)$(TNORMAL)"; \
		cp -a $(SRCDIR)/$*.h_ext $(IFACESRCDIR)/$*.h; \
		touch $(SRCDIR)/$*.cpp; \
	fi
    endif
  endif
	$(SILENT) mkdir -p $(@D)
	$(SILENT) touch $@

.SECONDARY: $(INTERFACES_SRCS) $(INTERFACES_HDRS) $(TOLUA_SRCS) $(TOLUA_ALL)

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
