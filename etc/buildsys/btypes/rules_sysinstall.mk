#*****************************************************************************
#     Makefile Build System for Fawkes: Rules for system-wide installation
#                            -------------------
#   Created on Mon Sep 07 11:24:07 2009
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/btypes/rules_fawkes.mk
include $(BUILDSYSDIR)/ext/gmsl

# Plugins are installed to special directory
$(foreach P,$(PLUGINS_all:$(PLUGINDIR)/%.so=%),$(eval INST_LIB_SUBDIR_$(subst /,_,$P) = $(FFLIBSUBDIR)/plugins))

# Library headers get subdir matching name if not set
$(foreach P,$(LIBS_all:$(LIBDIR)/libfawkes%.so=%) $(LIBS_gui:$(LIBDIR)/libfawkes%.so=%),$(if $(and $(call not,$(INST_HDRS_SUBDIR_libfawkes$P)),$P),$(eval INST_HDRS_SUBDIR_libfawkes$P = $P)))
$(foreach P,$(LIBS_all:$(LIBDIR)/libfv%.so=%) $(LIBS_gui:$(LIBDIR)/libfv%.so=%),$(if $(and $(call not,$(INST_HDRS_SUBDIR_libfv$P)),$P),$(eval INST_HDRS_SUBDIR_libfv$P = firevision/$P)))

ifdef __buildsys_lua_mk_
# Lua libraries are "inferred" and automatically installed to proper directory
$(foreach L,$(LIBS_all:$(LIBDIR)/lua/%.so=%),$(if $L,$(eval INST_LIB_SUBDIR_lua_$(subst /,_,$L) = $(FFLIBSUBDIR))))
endif

# Main install target
.PHONY: install install_test_basedir install_config install_buildsys install_lua
install: install_test_basedir presubdirs $(subst $(LIBDIR),$(EXEC_LIBDIR),$(LIBS_all) $(LIBS_gui)) $(subst $(PLUGINDIR),$(EXEC_PLUGINDIR),$(PLUGINS_all)) $(subst $(BINDIR),$(EXEC_BINDIR),$(BINS_all) $(BINS_gui)) resdirs subdirs install_config install_lua

# Only allow "make install" from basedir
install_test_basedir:
ifneq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
ifeq ($(INDENT),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- $(TRED)Installation may only be called from $(BASEDIR)$(TNORMAL)"
	$(SILENTSYMB)exit 1
endif
endif

.PHONY: resdirs $(INST_RESDIRS)
resdirs: $(INST_RESDIRS)

ifneq ($(INST_RESDIRS),)
$(INST_RESDIRS):
	$(SILENTSYMB) if [ -d "$(RESDIR)/$@" ]; then	\
		echo -e "$(INDENT_PRINT)--- Copying resource directory $@ to $(EXEC_RESDIR)/$@"; \
		mkdir -p $(EXEC_RESDIR)/$@ || exit $?; \
		cp -af $(RESDIR)/$@/* $(EXEC_RESDIR)/$@ || exit $$?; \
	fi
endif

install_config:
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Creating config directory $(EXEC_CONFDIR)"
	$(SILENT)mkdir -p $(EXEC_CONFDIR)
	$(SILENT)for f in $$(find cfg/ ! -name '*.db' -type f); do \
		if [ -e "$(EXEC_CONFDIR)/$${f/cfg\//}" ]; then \
			echo -e "$(INDENT_PRINT)--- $(TYELLOW)Omitting$(TNORMAL) config file $$f, already exists"; \
		else \
			echo -e "$(INDENT_PRINT)--- Copying config file $$f"; \
			install -D -m 644 $$f $(EXEC_CONFDIR)/$${f/cfg\//}; \
		fi \
	done
endif

install_buildsys:
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Creating buildsys directory $(EXEC_CONFDIR)"

install_lua:
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Creating Lua directory $(EXEC_LUADIR)"
	$(SILENT)mkdir -p $(EXEC_LUADIR)
	$(SILENT)for f in $$(find src/lua/ -name '*.lua'); do \
		echo -e "$(INDENT_PRINT)--- Copying Lua file $$f"; \
		install -D -m 644 $$f $(EXEC_LUADIR)/$${f/src\/lua\//}; \
	done
endif


# uninstall target to remove files from system
.PHONY: uninstall
uninstall: presubdirs subdirs
ifneq ($(BINS_all)$(BINS_gui),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Uninstalling binaries: $(subst $(BINDIR)/,,$(BINS_all) $(BINS_gui))"
	$(SILENT)rm -f $(subst $(BINDIR),$(EXEC_BINDIR),$(BINS_all) $(BINS_gui))
	$(SILENT)$(foreach B,$(subst $(BINDIR)/,,$(BINS_all) $(BINS_gui)),$(if $(wildcard $(EXEC_DFILEDIR)/$B.desktop),rm -f $(EXEC_DFILEDIR)/$B.desktop;))
endif
ifneq ($(LIBS_all)$(LIBS_gui),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Uninstalling libraries: $(subst $(LIBDIR)/,,$(LIBS_all) $(LIBS_gui))"
	$(SILENT)$(foreach L,$(subst $(LIBDIR)/,,$(LIBS_all) $(LIBS_gui)), \
	rm -f $(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$(L:%.so=%))))/$L*; \
	$(if $(HDRS_$(subst /,_,$(L:%.so=%))), \
	rm -f $(foreach h,$(HDRS_$(subst /,_,$(L:%.so=%))),"$(EXEC_INCDIR)/$(INST_HDRS_SUBDIR_$(subst /,_,$(L:%.so=%)))/$(if $(HDR_RENAME_$h),$(HDR_RENAME_$h),$h)" ); \
	))
endif
ifneq ($(PLUGINS_all)$(PLUGINS_gui),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Uninstalling plugins: $(subst $(PLUGINDIR)/,,$(PLUGINS_all) $(PLUGINS_gui))"
	$(SILENT)rm -f $(subst $(PLUGINDIR),$(EXEC_PLUGINDIR),$(PLUGINS_all) $(PLUGINS_gui))
endif
ifneq ($(INST_RESDIRS),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing resource direcotries: $(INST_RESDIRS)"
	$(SILENT)$(foreach D,$(INST_RESDIRS),rm -rf $(EXEC_RESDIR)/$D; )
endif
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing Fawkes library directory $(EXEC_FFLIBDIR)"
	$(SILENT)rm -rf $(EXEC_FFLIBDIR)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing Fawkes resource directory $(EXEC_RESDIR)"
	$(SILENT)rm -rf $(EXEC_RESDIR)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing Fawkes include directory $(EXEC_INCDIR)"
	$(SILENT)rm -rf $(EXEC_INCDIR)
endif



# xargs rmdir -p; \

# Library install target
# 1. Copy library, with or without SOVER
# 2. Copy header files
# Yes, the code is ugly, it does many calls to subst. That is necessary to make
# application Makefiles easy to read and write... Some of the most used patterns:
# $(subst /,_,$*)  Replace subdir separators with underscores, e.g. for
# $(LIBDIR)/interfaces/libMyInterface.so the stem becomes interfaces_libMyInterface.so
# SOVER related: generate different SOVER suffixes (e.g. 0.3.0, .0 and w/o suffix
# Path replacing: in app Makefiles project-relative paths are used, they have to
#                 be replaced by EXEC_ stuff. Additionall there are INST_LIB_SUBDIR
#                 items that allow libs to be installed in a subdir
$(EXEC_LIBDIR)/%.so: $(LIBDIR)/%.so
	$(if $(NOSOVER_$(subst /,_,$*)), \
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying library $* to $@"; \
	install -D $< $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<) || exit $$?; \
	, \
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying library $* to $@.$(SOVER_$(subst /,_,$*))"; \
	install -D $<.$(SOVER_$(subst /,_,$*)) $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(SOVER_$(subst /,_,$*)) || exit $$?; \
	echo -e "$(INDENT_PRINT)--- Creating symlink $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(firstword $(call split,.,$(SOVER_$(subst /,_,$*)))) -> $(notdir $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(SOVER_$(subst /,_,$*)))"; \
	ln -sf "$(notdir $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(SOVER_$(subst /,_,$*)))" "$(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(firstword $(call split,.,$(SOVER_$(subst /,_,$*))))" || exit $$?; \
	echo -e "$(INDENT_PRINT)--- Creating symlink $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<) -> $(notdir $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(SOVER_$(subst /,_,$*)))"; \
	ln -sf "$(notdir $(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<).$(SOVER_$(subst /,_,$*)))" "$(subst $(LIBDIR),$(abspath $(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(subst /,_,$*))),$<)" || exit $$?; \
	)
	$(SILENT) $(if $(HDRS_$(subst /,_,$*)),$(foreach h,$(HDRS_$(subst /,_,$*)), \
	if [ -f "$(SRCDIR)/$h" ]; then \
		echo -e "$(INDENT_PRINT)--- Copying header $h to $(EXEC_INCDIR)/$(INST_HDRS_SUBDIR_$(subst /,_,$*))/$(if $(HDR_RENAME_$h),$(HDR_RENAME_$h),$h)"; \
		install -D -m 644 "$(SRCDIR)/$h" "$(EXEC_INCDIR)/$(INST_HDRS_SUBDIR_$(subst /,_,$*))/$(if $(HDR_RENAME_$h),$(HDR_RENAME_$h),$h)" || exit $$?; \
	else \
		echo -e "$(INDENT_PRINT)--- $(TRED)Header $h does not exist.$(TNORMAL)"; \
		exit 1; \
	fi; \
	))

#	$(SILENTSYMB) for h in $(HDRS_$(subst /,_,$*)); do \

# Plugin install target
$(EXEC_PLUGINDIR)/%.so: $(PLUGINDIR)/%.so
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying plugin $* to $@"
	$(SILENT)install -D $< $@ || exit $$?

# Binary install target
$(EXEC_BINDIR)/%: $(BINDIR)/%
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying binary $* to $@"
	$(SILENT)install -D $< $@ || exit $$?
	$(SILENT)if [ -e "$(SRCDIR)/$*.desktop" ]; then \
		echo -e "$(INDENT_PRINT)--- Copying desktop file $*.desktop to $(EXEC_DFILEDIR)/$*.desktop"; \
		mkdir -p $(EXEC_DFILEDIR); \
		sed -e 's|@BASEDIR@|$(EXEC_BASEDIR)|' \
		-e 's|@CONFDIR@|$(SYSCONFDIR)|' -e 's|@RESDIR@|$(EXEC_RESDIR)|' \
		-e 's|@LIBDIR@|$(EXEC_LIBDIR)|' -e 's|@INCDIR@|$(EXEC_INCDIR)|' \
		-e 's|@IFACEDIR@|$(EXEC_IFACEDIR)|' \
		-e 's|@PLUGINDIR@|$(EXEC_PLUGINDIR)|' \
		$(SRCDIR)/$*.desktop > $(EXEC_DFILEDIR)/$*.desktop; \
		chmod 644 $(EXEC_DFILEDIR)/$*.desktop; \
	fi
