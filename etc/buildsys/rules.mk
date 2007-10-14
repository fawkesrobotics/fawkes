#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

# see http://make.paulandlesley.org/autodep.html
# see http://make.paulandlesley.org/rules.html

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
.PHONY: all
all: presubdirs $(LIBS_all) $(PLUGINS_all) $(BINS_all) $(TARGETS_all) subdirs

ifneq ($(OBJS_all),)
# Do not delete .o files to allow for incremental builds
.SECONDARY: $(OBJS_all)
# Whenever the Makefile is modified rebuild everything
$(OBJS_all): $(SRCDIR)/Makefile
else
  ifneq ($(LIBS_all)$(PLUGINS_all)$(BINS_all),)
    ifneq ($(DISABLE_OBJS_all_WARNING),1)
    $(warning OBJS_all is not set. This is probably a bug. If you intended this set DISABLE_OBJS_all_WARNING to 1 to get rid of this warning.)
    endif
  endif
endif

.PHONY: clean
clean: presubdirs subdirs
	$(SILENT) echo -e "$(INDENT_PRINT)--> Cleaning up directory $(TBOLDGRAY)$(CURDIR)$(TNORMAL)"
	$(SILENT) if [ "$(SRCDIR)/$(OBJDIR)" != "/" ]; then rm -rf $(SRCDIR)/$(OBJDIR) ; fi
	$(SILENT) if [ -n "$(DEPDIR)" ]; then rm -rf $(DEPDIR) ; fi
	$(SILENT) if [ -n "$(BINS_all)" ]; then rm -rf $(BINS_all) ; fi
	$(SILENT) if [ -n "$(LIBS_all)" ]; then rm -rf $(LIBS_all) ; fi
	$(SILENT) if [ -n "$(PLUGINS_all)" ]; then rm -rf $(PLUGINS_all) ; fi
	$(SILENT) if [ -n "$(TARGETS_all)" ]; then rm -rf $(TARGETS_all) ; fi

ifeq (,$(findstring qa,$(SUBDIRS)))
.PHONY: qa
qa: presubdirs subdirs
	$(SILENT) if [ -d "$(subst /.objs,,$(realpath $(CURDIR)))/qa" ]; then \
		echo -e "$(INDENT_PRINT)--> Building QA in $(subst $(realpath $(CURDIR)/$(BASEDIR))/,,$(subst /.objs,,$(realpath $(CURDIR)))/qa)"; \
		$(MAKE) --no-print-directory --no-keep-going -C "$(subst /.objs,,$(CURDIR))/qa" \
			SRCDIR="$(subst /.objs,,$(CURDIR))/qa" $(MFLAGS) INDENT="$(INDENT)$(INDENT_STRING)"; \
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
		$(MFLAGS) $(MAKECMDGOALS) INDENT="$(INDENT)$(INDENT_STRING)"; \
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
	$(SILENT) $(CC) -MD -MF $(df).td $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT)sed -e 's/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(df).td > $(df).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(df).td >> $(df).d; \
	rm -f $(df).td

%.o: %.c
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo "$(INDENT_PRINT)--- Compiling $(subst $(SRCDIR)/,,$<) (C)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -MD -MF $(df).td $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT)sed -e 's/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(df).td > $(df).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(df).td >> $(df).d; \
	rm -f $(df).td

moc_%.cpp: %.h
	$(SILENT) echo "$(INDENT_PRINT)--- Running Qt moc on $(subst $(SRCDIR)/,,$<), creating $(subst ..,__,$@)"
	$(SILENT) $(MOC) $(MOC_FLAGS) -p "../$(subst ..,__,$(@D))" $< -o $(subst ..,__,$@)

.SECONDEXPANSION:
$(BINDIR)/%: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(BINDIR)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) mkdir -p $(@D)
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

$(LIBDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(LIBDIR)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking lib $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

$(PLUGINDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(PLUGINDIR)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking plugin $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)


### Check if there are special additions
ifneq ($(realpath $(BASEDIR)/etc/buildsys_local/rules_$(BUILD_TYPE).mk),)
include $(BASEDIR)/etc/buildsys_local/rules_$(BUILD_TYPE).mk
endif

