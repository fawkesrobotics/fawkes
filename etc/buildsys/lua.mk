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

# List of acceptable Lua versions
LUA_VERSIONS= 5.2 5.1
$(foreach V,$(LUA_VERSIONS),$(if $(HAVE_LUA),,$(eval include $(BUILDSYSDIR)/lua_check.mk)))

ifeq ($(HAVE_LUA),1)
  ifeq ($(HAVE_TOLUA),1)
    CLEAN_FILES=*_tolua.{pkg,cpp}

.SECONDEXPANSION:
%_tolua.cpp: $$(TOLUA_$$(call nametr,$$*))
	$(SILENT) echo -e "$(INDENT_PRINT)[LUA] $(PARENTDIR)$(TBOLDGRAY)$(@F)$(TNORMAL)"
	$(SILENT)cat $(addprefix $(SRCDIR)/,$(subst $(SRCDIR)/,,$(filter %.tolua,$^))) > $(patsubst %.cpp,%.pkg,$@)
	$(SILENT)$(TOLUAPP) -L "$(FAWKES_BASEDIR)/src/lua/fawkes/toluaext.lua" -n $(TOLUA_PKGPREFIX_$(call nametr,$*))$(notdir $*) $(patsubst %.cpp,%.pkg,$@) | \
	sed -e 's/^\(.*Generated automatically .*\) on .*$$/\1/' | \
	awk '/^#if defined/ { f=1 }; f { t = t "\n" $$0 }; !f {print}; f && /^#endif/ {print "extern \"C\" {" t "\n}\n"; f=0}' | \
	awk '/^\*\/$$/ { print; while ((getline line < "$(BASEDIR)/doc/headers/lichead_c.GPL_WRE") > 0) print line; print "\n#include <core/exception.h>" }; ! /^\*\/$$/ { print }' \
	> $@

  endif # HAVE_TOLUA is 1
endif # HAVE_LUA is 1

ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_tolua_wrapper
warning_tolua_wrapper:
	$(SILENT)echo -e "$(INDENT_PRINT)[WARN] $(TRED)Omitting Lua compatibility wrapper in $(PARENTDIR)$(TNORMAL) (tolua++[-devel] not installed)"
endif # OBJSSUBMAKE is 1

endif # __buildsys_lua_mk_

