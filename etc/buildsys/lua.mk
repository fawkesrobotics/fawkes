#*****************************************************************************
#                 Makefile Build System for Fawkes: Lua bits
#                            -------------------
#   Created on Wed Mar 12 19:15:54 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_lua_mk_
__buildsys_lua_mk := 1

include $(BASEDIR)/etc/buildsys/config.mk

LUA_VERSION = 5.1

# Check for Lua (Fedora packages lua and lua-devel)
ifneq ($(PKGCONFIG),)
  HAVE_LUA = $(if $(shell $(PKGCONFIG) --atleast-version $(LUA_VERSION) 'lua'; echo $${?/1/}),1,0)
  LUA_LIBNAME = lua
  ifneq ($(HAVE_LUA),1)
    HAVE_LUA = $(if $(shell $(PKGCONFIG) --atleast-version $(LUA_VERSION) 'lua$(LUA_VERSION)'; echo $${?/1/}),1,0)
    LUA_LIBNAME = lua$(LUA_VERSION)
  endif
endif

ifeq ($(HAVE_LUA),1)
  LUADIR = $(abspath $(BASEDIR)/src/lua)
  LUALIBDIR = $(abspath $(LIBDIR)/lua)
  CFLAGS_LUA = $(shell $(PKGCONFIG) --cflags '$(LUA_LIBNAME)') -DLUADIR=\"$(LUADIR)\" -DLUALIBDIR=\"$(LUALIBDIR)\"
  LDFLAGS_LUA = $(shell $(PKGCONFIG) --libs '$(LUA_LIBNAME)')
  ifneq ($(wildcard $(SYSROOT)/usr/include/tolua++.h),)
    HAVE_TOLUA = 1
    TOLUAPP=tolua++
    TOLUA_LIBS=tolua++-$(LUA_VERSION)

.SECONDEXPANSION:
%_tolua.cpp: $$(TOLUA_$$(subst /,_,$$*))
	$(SILENT) echo "$(INDENT_PRINT)--- Generating Lua package C++ file $(@F)"
	$(SILENT)cat $(filter %.tolua,$^) | $(TOLUAPP) -n $(notdir $*) | \
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

