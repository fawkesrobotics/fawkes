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

LUA_MINVERSION = 5.1

# Check for Lua (Fedora packages lua and lua-devel)
ifneq ($(PKGCONFIG),)
  HAVE_LUA = $(if $(shell $(PKGCONFIG) --atleast-version $(LUA_MINVERSION) 'lua'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_LUA),1)
  CFLAGS_LUA = $(shell $(PKGCONFIG) --cflags 'lua')
  LDFLAGS_LUA = $(shell $(PKGCONFIG) --libs 'lua')
  ifneq ($(wildcard /usr/include/tolua++.h),)
    HAVE_TOLUA = 1
    TOLUAPP=tolua++

.SECONDEXPANSION:
%_tolua.cpp: $$(TOLUA_$$(subst /,_,$$*))
	$(SILENT) echo "$(INDENT_PRINT)--- Generating Lua package C++ file $(@F)"
	$(SILENT)cat $(filter %.tolua,$^) | $(TOLUAPP) -n $(notdir $*) | \
	sed -e 's/^\(.*Generated automatically .*\) on/\1/' | \
	awk '/^#if defined/ { f=1 }; f { t = t "\n" $$0 }; !f {print}; f && /^#endif/ {print "extern \"C\" {" t "\n}\n"; f=0}' \
	> $@

  endif # HAVE_TOLUA is 1
endif # HAVE_LUA is 1

ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_tolua_wrapper
warning_tolua_wrapper:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Lua compatibility wrapper$(TNORMAL) (tolua++[-devel] not installed)"
endif # OBJSSUBMAKE is 1

endif # __buildsys_lua_mk_

