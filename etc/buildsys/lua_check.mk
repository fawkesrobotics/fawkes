#*****************************************************************************
#                 Makefile Build System for Fawkes: Lua bits
#                            -------------------
#   Created on Mon Jan 05 16:13:22 2015
#   Copyright (C) 2006-2014 by Tim Niemueller
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
$(error lua.mk must be included before lua_check.mk)
endif

# Check for Lua (Fedora packages lua and lua-devel)
ifneq ($(PKGCONFIG),)
  __LUA_TRY_P := lua-$V lua$V lua

  LUA_PACKAGE := $(firstword $(foreach P,$(__LUA_TRY_P),$(if $(shell $(PKGCONFIG) --atleast-version $V $P; echo $${?/1/}),$P )))
  HAVE_LUA := $(if $(LUA_PACKAGE),1,)
endif

ifeq ($(HAVE_LUA),1)
  LUA_VERSION_SPLITTED := $(call split,.,$(shell $(PKGCONFIG) --modversion '$(LUA_PACKAGE)'))
  LUA_VERSION_MAJOR := $(word 1,$(LUA_VERSION_SPLITTED))
  LUA_VERSION_MINOR := $(word 2,$(LUA_VERSION_SPLITTED))
  LUA_VERSION := $(LUA_VERSION_MAJOR).$(LUA_VERSION_MINOR)

  ifeq ($(LUA_VERSION),$V)
    ifneq ($(wildcard $(SYSROOT)/usr/include/tolua++.h),)

      # Fedora 32 and 64 bit
      ifeq ($(ARCH),$(filter $(ARCH),x86_64 aarch64 ppc64 ppc64le s390x))
        ifneq ($(wildcard $(SYSROOT)/usr/lib64/libtolua++-$(LUA_VERSION).$(SOEXT)),)
          _HAVE_TOLUA_LIB := 1
          TOLUA_LIBS := tolua++-$(LUA_VERSION) stdc++
        endif
      else
        ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++-$(LUA_VERSION).$(SOEXT)),)
          _HAVE_TOLUA_LIB := 1
          TOLUA_LIBS := tolua++-$(LUA_VERSION) stdc++
        endif
      endif
      ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++),)
        _HAVE_TOLUA_BIN := 1
        TOLUAPP := tolua++
      endif

      # Ubuntu
      ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++$(LUA_VERSION).a),)
        _HAVE_TOLUA_LIB := 1
         TOLUA_LIBS := tolua++$(LUA_VERSION) stdc++ lua$(LUA_VERSION)
      endif

      # OpenEmbedded/Nao
      ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++-$(LUA_VERSION).a),)
        _HAVE_TOLUA_LIB := 1
        TOLUA_LIBS := tolua++-$(LUA_VERSION) stdc++
      endif
      ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++$(LUA_VERSION)),)
        _HAVE_TOLUA_BIN := 1
        TOLUAPP := tolua++$(LUA_VERSION)
      endif

      # Arch Linux
      ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++.a),)
        _HAVE_TOLUA_LIB := 1
        TOLUA_LIBS := tolua++ stdc++
      endif
      ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++),)
        _HAVE_TOLUA_BIN := 1
        TOLUAPP := tolua++
      endif

      # Gentoo (on x86_64, lib is a symlink to lib64)
      ifneq ($(wildcard $(SYSROOT)/usr/lib/libtolua++.$(SOEXT)),)
        _HAVE_TOLUA_LIB := 1
        TOLUA_LIBS := tolua++ stdc++
      endif
      ifneq ($(wildcard $(SYSROOT)/usr/bin/tolua++),)
        _HAVE_TOLUA_BIN := 1
        TOLUAPP := tolua++
      endif

      ifeq ($(_HAVE_TOLUA_LIB)$(_HAVE_TOLUA_BIN),11)
        HAVE_TOLUA  :=  1
      endif
    endif
    # FreeBSD
    ifneq ($(wildcard $(SYSROOT)/usr/local/include/lua$(subst .,,$(LUA_VERSION))/tolua++.h),)
      # FreeBSD
      HAVE_TOLUA := 1
      ifneq ($(wildcard $(SYSROOT)/usr/local/bin/lua$(subst .,,$(LUA_VERSION))/tolua++),)
        _HAVE_TOLUA_BIN := 1
        TOLUAPP := /usr/local/bin/lua$(subst .,,$(LUA_VERSION))/tolua++
      endif
      ifneq ($(wildcard $(SYSROOT)/usr/local/bin/tolua++),)
        _HAVE_TOLUA_BIN := 1
        TOLUAPP := /usr/local/bin/tolua++
      endif
      TOLUA_LIBS := tolua++ stdc++
    endif

    ifneq ($(HAVE_TOLUA),1)
      HAVE_LUA :=
      LUA_VERSION :=
    endif
  else
    HAVE_LUA :=
    LUA_VERSION :=
  endif
endif

ifeq ($(HAVE_LUA),1)
  LUADIR := $(abspath $(BASEDIR)/src/lua)
  LUALIBDIR := $(abspath $(LIBDIR)/lua)
  EXEC_LUADIR    ?= $(abspath $(EXEC_BASEDIR)/src/lua)
  EXEC_LUALIBDIR ?= $(abspath $(EXEC_LIBDIR)/lua)
  CFLAGS_LUA := $(shell $(PKGCONFIG) --cflags '$(LUA_PACKAGE)') \
	        -DHAVE_LUA -DLUADIR=\"$(EXEC_LUADIR)\" -DLUALIBDIR=\"$(EXEC_LUALIBDIR)\"
  LDFLAGS_LUA := $(shell $(PKGCONFIG) --libs '$(LUA_PACKAGE)')
endif
