#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

# see http://make.paulandlesley.org/autodep.html
# see http://make.paulandlesley.org/rules.html

ifndef __buildsys_rules_mk_
__buildsys_rules_mk := 1

include $(abspath $(BASEDIR)/etc/buildsys/ext/gmsl)

MAKE_MIN_VERSION_MAJOR=3
MAKE_MIN_VERSION_MINOR=81

MAKE_VERSION_SPLITTED=$(call split,.,$(MAKE_VERSION))
MAKE_VERSION_MAJOR=$(word 1,$(MAKE_VERSION_SPLITTED))
MAKE_VERSION_MINOR=$(word 2,$(MAKE_VERSION_SPLITTED))

ifneq ($(call gte,$(MAKE_VERSION_MAJOR),$(MAKE_MIN_VERSION_MAJOR)),$(true))
  MAKE_INSUFFICIENT=1
else
  ifeq ($(MAKE_VERSION_MAJOR),$(MAKE_MIN_VERSION_MAJOR))
    ifneq ($(call gte,$(MAKE_VERSION_MINOR),$(MAKE_MIN_VERSION_MINOR)),$(true))
      MAKE_INSUFFICIENT=1
    endif
  endif
endif

ifeq ($(MAKE_INSUFFICIENT),1)
  $(error You need at least GNU Make version $(MAKE_MIN_VERSION_MAJOR).$(MAKE_MIN_VERSION_MINOR), but you have only $(MAKE_VERSION))
endif

# indentation definitions, dash (-) is replaced with space
INDENT_STRING = ---
INDENT_PRINT := $(subst -, ,$(INDENT))
ifeq ($(MAKECMDGOALS),clean)
  DISABLE_OBJS_all_WARNING = 1
  ifeq ($(shell test -d qa; if [ "$$?" = "0" ]; then echo "yes"; fi),yes)
    ifneq ($(findstring qa,$(SUBDIRS)),qa)
      SUBDIRS += qa
    endif
  endif
endif

# Dependencies
-include $(DEPDIR)/*.d

# One to build 'em all
.PHONY: all gui
ifeq ($(MAKELEVEL),1)
  EXTRA_ALL = $(LIBS_gui) $(PLUGINS_gui) $(BINS_gui) $(TARGETS_gui)
endif
all: presubdirs $(LIBS_all) $(PLUGINS_all) $(BINS_all) $(TARGETS_all) $(EXTRA_ALL) subdirs
gui: presubdirs $(LIBS_gui) $(PLUGINS_gui) $(BINS_gui) $(TARGETS_gui) subdirs
uncolored-all: all
uncolored-gui: gui

ifdef OBJS_all
ifneq ($(OBJS_all),)
# Do not delete .o files to allow for incremental builds
.SECONDARY: $(OBJS_all)
# Whenever the Makefile is modified rebuild everything
$(OBJS_all): $(SRCDIR)/Makefile
endif
else
  ifneq ($(LIBS_all)$(PLUGINS_all)$(BINS_all)$(LIBS_gui)$(PLUGINS_gui)$(BINS_gui),)
    ifneq ($(DISABLE_OBJS_all_WARNING),1)
      $(warning OBJS_all is not set. This is probably a bug. If you intended this set DISABLE_OBJS_all_WARNING to 1 to get rid of this warning.)
    endif
  endif
endif


.PHONY: clean
clean: presubdirs subdirs
	$(SILENT) echo -e "$(INDENT_PRINT)--> Cleaning up directory $(TBOLDGRAY)$(CURDIR)$(TNORMAL)"
	$(SILENT) if [ "$(SRCDIR)/$(OBJDIR)" != "/" ]; then rm -rf "$(SRCDIR)/$(OBJDIR)" ; fi
	$(SILENT) if [ -n "$(DEPDIR)" ]; then rm -rf "$(DEPDIR)" ; fi
	$(SILENT)$(foreach B,$(BINS_all),rm -rf $(B);)
	$(SILENT)$(foreach L,$(LIBS_all),rm -rf $(L);)
	$(SILENT)$(foreach P,$(PLUGINS_all),rm -rf $(P);)
	$(SILENT)$(foreach T,$(TARGETS_all),rm -rf $(T);)
	$(SILENT)$(foreach B,$(BINS_gui),rm -rf $(B);)
	$(SILENT)$(foreach L,$(LIBS_gui),rm -rf $(L);)
	$(SILENT)$(foreach P,$(PLUGINS_gui),rm -rf $(P);)
	$(SILENT)$(foreach T,$(TARGETS_gui),rm -rf $(T);)

ifeq (,$(findstring qa,$(SUBDIRS)))
.PHONY: qa
qa: presubdirs subdirs
	$(SILENT) if [ -d "$(subst /.objs,,$(realpath $(CURDIR)))/qa" ]; then \
		echo -e "$(INDENT_PRINT)--> Building QA in $(subst $(realpath $(CURDIR)/$(BASEDIR))/,,$(subst /.objs,,$(realpath $(CURDIR)))/qa)"; \
		$(MAKE) --no-print-directory --no-keep-going -C "$(subst /.objs,,$(CURDIR))/qa" \
			SRCDIR="$(subst /.objs,,$(CURDIR))/qa" $(MFLAGS) INDENT="$(INDENT)$(INDENT_STRING)" || exit $$?; \
	fi
endif

.PHONY: presubdirs $(PRESUBDIRS) subdirs $(SUBDIRS)
presubdirs: $(PRESUBDIRS)
subdirs: $(SUBDIRS)

ifneq ($(PRESUBDIRS)$(SUBDIRS),)
$(PRESUBDIRS) $(SUBDIRS):
	$(SILENT) if [ ! -d "$(realpath $(SRCDIR)/$(@))" ]; then \
		echo -e "$(INDENT_PRINT)---$(TRED)Directory $(TNORMAL)$(TBOLDRED)$@$(TNORMAL)$(TRED) does not exist, check [PRE]SUBDIRS variable$(TNORMAL) ---"; \
		exit 1; \
	else \
		echo -e "$(INDENT_PRINT)--> Entering sub-directory $(TBOLDBLUE)$@$(TNORMAL) ---"; \
		$(MAKE) --no-print-directory --no-keep-going -C "$(realpath $(SRCDIR)/$@)" \
		$(MFLAGS) $(MAKECMDGOALS) INDENT="$(INDENT)$(INDENT_STRING)" \
		SRCDIR="$(realpath $(SRCDIR)/$@)" || exit $$?; \
		if [ "$(MAKECMDGOALS)" != "clean" ]; then \
			echo -e "$(INDENT_PRINT)$(subst -, ,$(INDENT_STRING))<-- Leaving $@"; \
		fi \
	fi
endif

# Cancel built-in implicit rules, they suck
%: %.cpp
%: %.c

# NOTE: .. are replaced by __. This is needed to have objects that are in upper relative
# directories are build in .objs, another change is needed below in bin, lib and plugin targets,
# mocs etc.
%.o: %.cpp
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo "$(INDENT_PRINT)--- Compiling $(subst $(SRCDIR)/,,$<) (C++)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -MD -MF $(DEPFILE).td $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT)sed -e '/^[^:]*\//! s/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(DEPFILE).td > $(DEPFILE).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEPFILE).td >> $(DEPFILE).d; \
	rm -f $(DEPFILE).td

%.o: %.c
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo "$(INDENT_PRINT)--- Compiling $(subst $(SRCDIR)/,,$<) (C)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -MD -MF $(DEPFILE).td $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT)sed -e '/^[^:]*\//! s/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(DEPFILE).td > $(DEPFILE).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEPFILE).td >> $(DEPFILE).d; \
	rm -f $(DEPFILE).td

moc_%.cpp: %.h
	$(SILENT) echo "$(INDENT_PRINT)--- Running Qt moc on $(subst $(SRCDIR)/,,$<), creating $(subst ..,__,$@)"
	$(SILENT) $(MOC) $(MOC_FLAGS) -p "../$(subst ..,__,$(@D))" $< -o $(subst ..,__,$@)

.SECONDEXPANSION:
$(BINDIR)/%: $$(OBJS_$$(subst /,_,$$*))
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) -o $@ $(subst ..,__,$^) \
	$(LDFLAGS_BASE) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$(subst /,_,$*)) \
	$(addprefix -l,$(LIBS_$(subst /,_,$*))) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$(subst /,_,$*))) $(addprefix -L,$(LIBDIRS))

$(LIBDIR)/%.so: $$(OBJS_$$(subst /,_,$$*))
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking lib $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) -o $@ $(subst ..,__,$^) \
	$(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$(subst /,_,$*)) \
	$(addprefix -l,$(LIBS_$(subst /,_,$*))) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$(subst /,_,$*))) $(addprefix -L,$(LIBDIRS))


### Check if there are special additions
ifneq ($(realpath $(BASEDIR)/etc/buildsys/btypes/rules_$(BUILD_TYPE).mk),)
include $(BASEDIR)/etc/buildsys/btypes/rules_$(BUILD_TYPE).mk
endif

endif # __buildsys_rules_mk_

