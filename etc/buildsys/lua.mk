#*****************************************************************************
#                 Makefile Build System for Fawkes: Lua bits
#                            -------------------
#   Created on Wed Mar 12 19:15:54 2008
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

ifndef __buildsys_config_mk_
$(error config.mk must be included before lua.mk)
endif

ifndef __buildsys_lua_mk_
__buildsys_lua_mk_ := 1

LUA_VERSION = 5.1

# Check for Lua (Fedora packages lua and lua-devel)
ifneq ($(PKGCONFIG),)
  LUA_PACKAGE = $(firstword $(foreach P,lua$(LUA_VERSION) lua-$(LUA_VERSION) lua,$(if $(shell $(PKGCONFIG) --atleast-version $(LUA_VERSION) $(P); echo $${?/1/}),$(P))))
  HAVE_LUA = $(if $(LUA_PACKAGE),1,0)
endif

ifeq ($(HAVE_LUA),1)
  LUADIR = $(abspath $(BASEDIR)/src/lua)
  LUALIBDIR = $(abspath $(LIBDIR)/lua)
  EXEC_LUADIR    ?= $(abspath $(EXEC_BASEDIR)/src/lua)
  EXEC_LUALIBDIR ?= $(abspath $(EXEC_LIBDIR)/lua)
  CFLAGS_LUA = $(shell $(PKGCONFIG) --cflags '$(LUA_PACKAGE)') -DHAVE_LUA -DLUADIR=\"$(EXEC_LUADIR)\" -DLUALIBDIR=\"$(EXEC_LUALIBDIR)\"
  LDFLAGS_LUA = $(shell $(PKGCONFIG) --libs '$(LUA_PACKAGE)')
  ifneq ($(wildcard $(SYSROOT)/usr/include/tolua++.h),)

    # Fedora 32 and 64 bit
    ifeq ($(ARCH),x86_64)
      ifneq ($(wildcard $(SYSROOT)/usr/lib64/libtolua++-$(LUA_VERSION).$(SOEXT)),)
        _HAVE_TOLUA_LIB=1
        TOLUA_LIBS=tolua++-$(LUA_VERSION) stdc++
      endif
    else
      ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++-$(LUA_VERSION).$(SOEXT)),)
        _HAVE_TOLUA_LIB=1
        TOLUA_LIBS=tolua++-$(LUA_VERSION) stdc++
      endif
    endif
    ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++),)
      _HAVE_TOLUA_BIN=1
      TOLUAPP=tolua++
    endif
    
    # Ubuntu
    ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++$(LUA_VERSION).a),)
      _HAVE_TOLUA_LIB=1
       TOLUA_LIBS=tolua++$(LUA_VERSION) stdc++ lua$(LUA_VERSION)
    endif

    # OpenEmbedded/Nao
    ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++-$(LUA_VERSION).a),)
      _HAVE_TOLUA_LIB=1
       TOLUA_LIBS=tolua++-$(LUA_VERSION) stdc++
    endif
    ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++$(LUA_VERSION)),)
      _HAVE_TOLUA_BIN=1
      TOLUAPP=tolua++$(LUA_VERSION)
    endif

    # Arch Linux
    ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++.a),)
      _HAVE_TOLUA_LIB=1
      TOLUA_LIBS=tolua++ stdc++
    endif
    ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++),)
      _HAVE_TOLUA_BIN=1
      TOLUAPP=tolua++
    endif

    # Gentoo (on x86_64, lib is a symlink to lib64)
    ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++.$(SOEXT)),)
      _HAVE_TOLUA_LIB=1
      TOLUA_LIBS=tolua++ stdc++
    endif
    ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++),)
      _HAVE_TOLUA_BIN=1
      TOLUAPP=tolua++
    endif

    ifeq ($(_HAVE_TOLUA_LIB)$(_HAVE_TOLUA_BIN),11)
      HAVE_TOLUA = 1
    endif
  endif
  # FreeBSD
  ifneq ($(wildcard $(SYSROOT)/usr/local/include/lua$(subst .,,$(LUA_VERSION))/tolua++.h),)
    # FreeBSD
    HAVE_TOLUA = 1
    TOLUAPP=/usr/local/bin/lua$(subst .,,$(LUA_VERSION))/tolua++
    TOLUA_LIBS=tolua++ stdc++
  endif
  CLEAN_FILES=*_tolua.{pkg,cpp}

  ifeq ($(HAVE_TOLUA),1)
.SECONDEXPANSION:
%_tolua.cpp: $$(TOLUA_$$(call nametr,$$*))
	$(SILENT) echo "$(INDENT_PRINT)--- Generating Lua package C++ file $(@F)"
	$(SILENT)cat $(addprefix $(SRCDIR)/,$(subst $(SRCDIR)/,,$(filter %.tolua,$^))) > $(patsubst %.cpp,%.pkg,$@)
	$(SILENT)$(TOLUAPP) -n $(TOLUA_PKGPREFIX_$(call nametr,$*))$(notdir $*) $(patsubst %.cpp,%.pkg,$@) | \
	sed -e 's/^\(.*Generated automatically .*\) on .*$$/\1/' | \
	awk '/^#if defined/ { f=1 }; f { t = t "\n" $$0 }; !f {print}; f && /^#endif/ {print "extern \"C\" {" t "\n}\n"; f=0}' | \
	awk '/^\*\/$$/ { print; while ((getline line < "$(BASEDIR)/doc/headers/lichead_c.GPL_WRE") > 0) print line }; ! /^\*\/$$/ { print }' \
	> $@

  endif # HAVE_TOLUA is 1
endif # HAVE_LUA is 1

ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_tolua_wrapper
warning_tolua_wrapper:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Lua compatibility wrapper$(TNORMAL) (tolua++[-devel] not installed)"
endif # OBJSSUBMAKE is 1

endif # __buildsys_lua_mk_

