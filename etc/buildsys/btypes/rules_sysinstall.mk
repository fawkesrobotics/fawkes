#*****************************************************************************
#     Makefile Build System for Fawkes: Rules for system-wide installation
#                            -------------------
#   Created on Mon Sep 07 11:24:07 2009
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(foreach P,$(PLUGINS_build:$(PLUGINDIR)/%.so=%),$(eval INST_LIB_SUBDIR_$(call nametr,$P) = $(FFLIBSUBDIR)/plugins))

# Library headers get subdir matching name if not set
$(foreach P,$(LIBS_build:$(LIBDIR)/libfawkes%.so=%) $(LIBS_gui:$(LIBDIR)/libfawkes%.so=%),$(if $(and $(call not,$(INST_HDRS_SUBDIR_libfawkes$P)),$P),$(eval INST_HDRS_SUBDIR_libfawkes$P = $(P:_%=%))))
$(foreach P,$(LIBS_build:$(LIBDIR)/libfv%.so=%) $(LIBS_gui:$(LIBDIR)/libfv%.so=%),$(if $(and $(call not,$(INST_HDRS_SUBDIR_libfv$P)),$P),$(eval INST_HDRS_SUBDIR_libfv$P = fv$P)))

ifdef __buildsys_lua_mk_
# Lua libraries are "inferred" and automatically installed to proper directory
$(foreach L,$(LIBS_build:$(LIBDIR)/lua/%.so=%),$(if $L,$(eval INST_LIB_SUBDIR_lua_$(call nametr,$L) = $(FFLIBSUBDIR))))
endif

# Prefix man pages with proper path
MANPAGES_install = $(addprefix $(DESTDIR)$(EXEC_MANDIR)/,$(patsubst $(abspath $(MANDIR))/%,%,$(MANPAGES_all) $(MANPAGES_gui)))

# Main install target
.PHONY: install install_test_basedir install_config install_buildsys install_lua install_apidoc uncolored-install
uncolored-install: install
install: install_test_basedir presubdirs install_targets install_resdirs install_extra subdirs install_buildsys install_config install_lua install_apidoc
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENT)echo -e "$(TGREEN)*** Installation completed ***$(TNORMAL)"
endif

# Only allow "make install" from basedir
install_test_basedir:
ifneq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
ifeq ($(INDENT),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- $(TRED)Installation may only be called from $(BASEDIR)$(TNORMAL)"
	$(SILENTSYMB)exit 1
endif
endif

.PHONY: install_targets
install_targets: $(subst $(LIBDIR),$(DESTDIR)$(EXEC_LIBDIR),$(LIBS_build) $(LIBS_gui)) $(subst $(PLUGINDIR),$(DESTDIR)$(EXEC_PLUGINDIR),$(PLUGINS_build)) $(subst $(BINDIR),$(DESTDIR)$(EXEC_BINDIR),$(BINS_build) $(BINS_gui)) $(MANPAGES_install)

.PHONY: install_resdirs $(INST_RESDIRS)
install_resdirs: $(INST_RESDIRS)

.PHONY: install_extra $(INSTALL_extra)
install_extra: $(INSTALL_extra)

ifneq ($(INSTALL_extra),1)
$(INSTALL_extra): %: $$(FILES_$$(call nametr,$$*))
	$(SILENTSYMB)for f in $(subst $(SRCDIR)/,,$^); do \
		if [ ! -d $(DESTDIR)$(DESTDIR_$*) ]; then \
			echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(DESTDIR_$*)$(TNORMAL)"; \
			mkdir -p "$(DESTDIR)$(DESTDIR_$*)"; \
		fi; \
		echo -e "$(INDENT_PRINT)[CPY] $(PARENTDIR)$(TBOLDGRAY)$$f$(TNORMAL) -> $(DESTDIR)$(DESTDIR_$*)"; \
		$(FILE_INSTALL) -m $(if $(FILEMODE_$*),$(FILEMODE_$*),$(FILEMODE_DEFAULT)) -t $(DESTDIR)$(DESTDIR_$*) $(SRCDIR)/$$f; \
	done
endif

ifneq ($(INST_RESDIRS),)
$(INST_RESDIRS):
	$(SILENTSYMB) if [ -d "$(RESDIR)/$@" ]; then	\
		echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(RESDIR)/$@$(TNORMAL)"; \
		mkdir -p $(DESTDIR)$(EXEC_RESDIR)/$@ || exit $?; \
		echo -e "$(INDENT_PRINT)[CPY] $(PARENTDIR)$(TBOLDGRAY)$@$(TNORMAL) -> $(DESTDIR)$(EXEC_RESDIR)/$@"; \
		cp -af $(RESDIR)/$@/* $(DESTDIR)$(EXEC_RESDIR)/$@ || exit $$?; \
	fi
endif

install_config:
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENT)echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(EXEC_CONFDIR)$(TNORMAL)"
	$(SILENT)mkdir -p $(DESTDIR)$(EXEC_CONFDIR)
	$(SILENT)for f in $$(find cfg/ ! -name '*.db' -type f); do \
		if [ -e "$(DESTDIR)$(EXEC_CONFDIR)/$${f/cfg\//}" ]; then \
			echo -e "$(INDENT_PRINT)[CPY] $(TYELLOW)Omitting config file $(TNORMAL)$(PARENTDIR)$(TBOLDGRAY)$$f$(TNORMAL) (file already exists)"; \
		else \
			echo -e "$(INDENT_PRINT)[CPY] $(PARENTDIR)$(TBOLDGRAY)$$f$(TNORMAL) -> $(DESTDIR)$(EXEC_CONFDIR)/$${f/cfg\//}"; \
			install -D -m 644 $$f $(DESTDIR)$(EXEC_CONFDIR)/$${f/cfg\//}; \
		fi \
	done
endif

install_buildsys:
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENT)echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(EXEC_BUILDSYSDIR)$(TNORMAL)"
	$(SILENT)mkdir -p $(DESTDIR)$(EXEC_BUILDSYSDIR)
	$(SILENT)for f in $$(find $(BUILDSYSDIR) -type d -printf "%P\n"); do \
		if [ "$$F" == "" ]; then continue; fi; \
		mkdir -p $(BUILDSYSDIR)/$$f; \
	done
	$(SILENT)for f in $$(find $(BUILDSYSDIR) -type f ! -regex '.*[\~#]$$' -printf "%P\n"); do \
		echo -e "$(INDENT_PRINT)[CPY] $(subst $(realpath $(TOP_BASEDIR))/,,$(BUILDSYSDIR))/$(TBOLDGRAY)$$f$(TNORMAL) -> $(DESTDIR)$(EXEC_BUILDSYSDIR)/$$f"; \
		install -D -m 644 $(BUILDSYSDIR)/$$f $(DESTDIR)$(EXEC_BUILDSYSDIR)/$$f; \
	done
	$(SILENT)echo -e "$(INDENT_PRINT)--- Setting installed build type to 'syswide'";
	$(SILENT)sed -i -e 's/^BUILD_TYPE=.*$$/BUILD_TYPE=syswide/' $(DESTDIR)$(EXEC_BUILDSYSDIR)/buildtype.mk
	$(SILENT)echo -e "$(INDENT_PRINT)--- Setting installed INSTALL_PREFIX to '$(PREFIX)'";
	$(SILENT)sed -i -e 's|^INSTALL_PREFIX\( *\)=.*$$|INSTALL_PREFIX\1= $(PREFIX)|' $(DESTDIR)$(EXEC_BUILDSYSDIR)/btypes/config_syswide.mk
	$(SILENT)echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(EXEC_BUILDCONFDIR)$(TNORMAL)"
	$(SILENT)find $(BUILDCONFDIR) -name '*.mk' -type f -printf "%f:%P\n" | \
		while IFS=":" read basename relname; \
		do \
			echo -e "$(INDENT_PRINT)[CPY] $(subst $(realpath $(TOP_BASEDIR))/,,$(BUILDCONFDIR))/$(TBOLDGRAY)$$relname$(TNORMAL) -> $(DESTDIR)$(EXEC_BUILDSYSDIR)/$$basename"; \
			install -D -m 644 $(BUILDCONFDIR)/$$relname $(DESTDIR)$(EXEC_BUILDCONFDIR)/$$basename; \
		done

endif

install_lua:
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENT)echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(EXEC_LUADIR)$(TNORMAL)"
	$(SILENT)mkdir -p $(DESTDIR)$(EXEC_LUADIR)
	$(SILENT)for f in $$(find src/lua/ -name '*.lua'); do \
		echo -e "$(INDENT_PRINT)[CPY] $(PARENTDIR)$(TBOLDGRAY)$$f$(TNORMAL) -> $(DESTDIR)$(EXEC_LUADIR)/$${f/src\/lua\//}"; \
		install -D -m 644 $$f $(DESTDIR)$(EXEC_LUADIR)/$${f/src\/lua\//}; \
	done
endif

install_apidoc:
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Creating documentation directory $(DESTDIR)$(EXEC_DOCDIR)"
	$(SILENT)if [ -d "$(DOCDIR)/api/html" ]; then \
		echo -e "$(INDENT_PRINT)[DIR] $(DESTDIR)$(TBOLDGRAY)$(EXEC_DOCDIR)$(TNORMAL)"; \
		mkdir -p $(DESTDIR)$(EXEC_DOCDIR); \
		echo -e "$(INDENT_PRINT)[CPY] $(DOCDIR)/api/html/* -> $(DESTDIR)$(EXEC_DOCDIR)"; \
		cp -ar $(DOCDIR)/api/html/* $(DESTDIR)$(EXEC_DOCDIR); \
	else \
		echo -e "$(INDENT_PRINT)--- $(TYELLOW)API documentation not generated, not copying$(TNORMAL)"; \
	fi
endif


# uninstall target to remove files from system
.PHONY: uninstall
uninstall: presubdirs subdirs
ifneq ($(BINS_build)$(BINS_gui),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Uninstalling binaries: $(subst $(BINDIR)/,,$(BINS_build) $(BINS_gui))"
	$(SILENT)rm -f $(subst $(BINDIR),$(DESTDIR)$(EXEC_BINDIR),$(BINS_build) $(BINS_gui))
	$(SILENT)$(foreach B,$(subst $(BINDIR)/,,$(BINS_build) $(BINS_gui)),$(if $(wildcard $(DESTDIR)$(EXEC_DFILEDIR)/$B.desktop),rm -f $(DESTDIR)$(EXEC_DFILEDIR)/$B.desktop;))
endif
ifneq ($(LIBS_build)$(LIBS_gui),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Uninstalling libraries: $(subst $(LIBDIR)/,,$(LIBS_build) $(LIBS_gui))"
	$(SILENT)$(foreach L,$(subst $(LIBDIR)/,,$(LIBS_build) $(LIBS_gui)), \
	rm -f $(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$(L:%.so=%))))/$L*; \
	$(if $(HDRS_$(call nametr,$(L:%.so=%))), \
	rm -f $(foreach h,$(HDRS_$(call nametr,$(L:%.so=%))),"$(DESTDIR)$(EXEC_INCDIR)/$(INST_HDRS_SUBDIR_$(call nametr,$(L:%.so=%)))/$(if $(HDR_RENAME_$h),$(HDR_RENAME_$h),$h)" ); \
	))
endif
ifneq ($(PLUGINS_build)$(PLUGINS_gui),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Uninstalling plugins: $(subst $(PLUGINDIR)/,,$(PLUGINS_build) $(PLUGINS_gui))"
	$(SILENT)rm -f $(subst $(PLUGINDIR),$(DESTDIR)$(EXEC_PLUGINDIR),$(PLUGINS_build) $(PLUGINS_gui))
endif
ifneq ($(INST_RESDIRS),)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing resource direcotries: $(INST_RESDIRS)"
	$(SILENT)$(foreach D,$(INST_RESDIRS),rm -rf $(DESTDIR)$(EXEC_RESDIR)/$D; )
endif
ifeq ($(abspath $(SRCDIR)),$(abspath $(BASEDIR)))
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing Fawkes library directory $(DESTDIR)$(EXEC_FFLIBDIR)"
	$(SILENT)rm -rf $(DESTDIR)$(EXEC_FFLIBDIR)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing Fawkes resource directory $(DESTDIR)$(EXEC_RESDIR)"
	$(SILENT)rm -rf $(DESTDIR)$(EXEC_RESDIR)
	$(SILENTSYMB)echo -e "$(INDENT_PRINT)--- Removing Fawkes include directory $(DESTDIR)$(EXEC_INCDIR)"
	$(SILENT)rm -rf $(DESTDIR)$(EXEC_INCDIR)
endif

# Library install target
# 1. Copy library, with or without SOVER
# 2. Copy header files
# Yes, the code is ugly, it does many calls to subst. That is necessary to make
# application Makefiles easy to read and write... Some of the most used patterns:
# $(call nametr,$*)  Replace subdir separators with underscores, e.g. for
# $(LIBDIR)/interfaces/libMyInterface.so the stem becomes interfaces_libMyInterface.so
# SOVER related: generate different SOVER suffixes (e.g. 0.3.0, .0 and w/o suffix
# Path replacing: in app Makefiles project-relative paths are used, they have to
#                 be replaced by EXEC_ stuff. Additionall there are INST_LIB_SUBDIR
#                 items that allow libs to be installed in a subdir
$(DESTDIR)$(EXEC_LIBDIR)/%.so: $(LIBDIR)/%.so
	$(if $(NOSOVER_$(call nametr,$*)), \
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying library $* to $@"; \
	install -D $< $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<) || exit $$?; \
	, \
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying library $* to $@.$(SOVER_$(call nametr,$*))"; \
	install -D $<.$(SOVER_$(call nametr,$*)) $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(SOVER_$(call nametr,$*)) || exit $$?; \
	echo -e "$(INDENT_PRINT)--- Creating symlink $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(firstword $(call split,.,$(SOVER_$(call nametr,$*)))) -> $(notdir $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(SOVER_$(call nametr,$*)))"; \
	ln -sf "$(notdir $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(SOVER_$(call nametr,$*)))" "$(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(firstword $(call split,.,$(SOVER_$(call nametr,$*))))" || exit $$?; \
	echo -e "$(INDENT_PRINT)--- Creating symlink $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<) -> $(notdir $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(SOVER_$(call nametr,$*)))"; \
	ln -sf "$(notdir $(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<).$(SOVER_$(call nametr,$*)))" "$(subst $(LIBDIR),$(abspath $(DESTDIR)$(EXEC_LIBDIR)/$(INST_LIB_SUBDIR_$(call nametr,$*))),$<)" || exit $$?; \
	)
	$(SILENT) $(if $(HDRS_$(call nametr,$*)),$(foreach h,$(HDRS_$(call nametr,$*)), \
	if [ -f "$(SRCDIR)/$h" ] ; then \
		hpath=$(SRCDIR)/$h; \
	elif [ -f "$(IFACESRCDIR)/$h" ]; then \
		hpath=$(IFACESRCDIR)/$h; \
	else \
		echo -e "$(INDENT_PRINT)--- $(TRED)Header $h does not exist.$(TNORMAL)"; \
		exit 1; \
	fi; \
	echo -e "$(INDENT_PRINT)--- Copying header $h to $(DESTDIR)$(EXEC_INCDIR)/$(INST_HDRS_SUBDIR_$(call nametr,$*))/$(if $(HDR_RENAME_$h),$(HDR_RENAME_$h),$h)"; \
	install -D -m 644 "$$hpath" "$(DESTDIR)$(EXEC_INCDIR)/$(INST_HDRS_SUBDIR_$(call nametr,$*))/$(if $(HDR_RENAME_$h),$(HDR_RENAME_$h),$h)" || exit $$?; \
	))

#	$(SILENTSYMB) for h in $(HDRS_$(call nametr,$*)); do \

# Plugin install target
$(DESTDIR)$(EXEC_PLUGINDIR)/%.so: $(PLUGINDIR)/%.so
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying plugin $* to $@"
	$(SILENT)install -D $< $@ || exit $$?

# Binary install target
$(DESTDIR)$(EXEC_BINDIR)/%: $(BINDIR)/%
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Copying binary $* to $@"
	$(SILENT)install -D $< $@ || exit $$?
	$(SILENT)if [ -e "$(SRCDIR)/$*.desktop" ]; then \
		echo -e "$(INDENT_PRINT)--- Copying desktop file $*.desktop to $(DESTDIR)$(EXEC_DFILEDIR)/$*.desktop"; \
		mkdir -p $(DESTDIR)$(EXEC_DFILEDIR); \
		sed -e 's|@BASEDIR@|$(EXEC_BASEDIR)|' \
		-e 's|@CONFDIR@|$(SYSCONFDIR)|' -e 's|@RESDIR@|$(EXEC_RESDIR)|' \
		-e 's|@LIBDIR@|$(EXEC_LIBDIR)|' -e 's|@INCDIR@|$(EXEC_INCDIR)|' \
		-e 's|@IFACEDIR@|$(EXEC_IFACEDIR)|' \
		-e 's|@PLUGINDIR@|$(EXEC_PLUGINDIR)|' \
		$(SRCDIR)/$*.desktop > $(DESTDIR)$(EXEC_DFILEDIR)/$*.desktop; \
		chmod 644 $(DESTDIR)$(EXEC_DFILEDIR)/$*.desktop; \
	fi


# Manpage install target
$(DESTDIR)$(EXEC_MANDIR)/%: $(MANDIR)/%
	$(SILENT)if [ -e "$<" ]; then \
		echo -e "$(INDENT_PRINT)--- Copying man page $* to $@"; \
		install -D -m 0644  $< $@ || exit $$?; \
	else \
		echo -e "$(INDENT_PRINT)$(TYELLOW)--- Cannot copy man page $* (does not exist)$(TNORMAL)"; \
	fi
