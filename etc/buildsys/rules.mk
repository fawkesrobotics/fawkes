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
  ifeq ($(shell test -d qa; if [ "$$?" = "0" ]; then echo "yes"; fi),yes)
    ifneq ($(findstring qa,$(SUBDIRS)),qa)
      SUBDIRS += qa
    endif
  endif
endif

-include $(DEPDIR)/*.d

.SECONDARY: $(OBJS_all)

.PHONY: all
all: $(LIBS_all) $(PLUGINS_all) $(BINS_all) subdirs

.PHONY: clean
clean: subdirs
	$(SILENT) echo -e "$(INDENT_PRINT)--> Cleaning up directory $(TBOLDGRAY)$(CURDIR)$(TNORMAL)"
	$(SILENT) if [ "$(SRCDIR)/$(OBJDIR)" != "/" ]; then rm -rf $(SRCDIR)/$(OBJDIR) ; fi
	$(SILENT) if [ "$(DEPDIR)" != "" ]; then rm -rf $(DEPDIR) ; fi
	$(SILENT) if [ "$(BINS_all)" != "" ]; then rm -rf $(BINS_all) ; fi
	$(SILENT) if [ "$(LIBS_all)" != "" ]; then rm -rf $(LIBS_all) ; fi
	$(SILENT) if [ "$(PLUGINS_all)" != "" ]; then rm -rf $(PLUGINS_all) ; fi

ifeq (,$(findstring qa,$(SUBDIRS)))
.PHONY: qa
qa: subdirs
	$(SILENT) if [ -d "$(subst /.objs,,$(realpath $(CURDIR)))/qa" ]; then \
		echo -e "$(INDENT_PRINT)--> Building QA in $(subst $(realpath $(CURDIR)/$(BASEDIR))/,,$(subst /.objs,,$(realpath $(CURDIR)))/qa)"; \
		$(MAKE) --no-print-directory -C $(subst /.objs,,$(CURDIR))/qa INDENT="$(INDENT)$(INDENT_STRING)"; \
	fi
endif

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)

ifneq ($(SUBDIRS),)
$(SUBDIRS):
	$(SILENT) echo -e "\n$(INDENT_PRINT)--- Entering sub-directory $(TBOLDBLUE)$@$(TNORMAL) ---"
	$(SILENT) $(MAKE) --no-print-directory -C $(realpath $(SRCDIR)/$@) $(MAKECMDGOALS) INDENT="$(INDENT)$(INDENT_STRING)"
	$(SILENT) if [ "$(MAKECMDGOALS)" != "clean" ]; then \
		echo -e "$(INDENT_PRINT)$(subst -, ,$(INDENT_STRING))<-- Leaving $@"; \
	fi
endif

# NOTE: .. are replaced by __. This is needed to have objects that are in upper relative
# directories are build in .objs, another change is needed below in bin, lib and plugin targets,
# mocs etc.
%.o: %.cpp
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo "$(INDENT_PRINT)--> Compiling $(subst $(SRCDIR)/,,$<) (C++)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -Wp,-M,-MF,$(df).d $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT) cp -f $(df).d $(df).td; \
	sed -e 's/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(df).td > $(df).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(df).td >> $(df).d; \
	rm -f $(df).td

# Make .cpp files depend on Makefile to notice changes to Makefile (like config changes)
%.cpp: $(SRCDIR)/Makefile

moc_%.cpp: %.h
	$(SILENT) echo "$(INDENT_PRINT)--- Running Qt moc on $(subst $(SRCDIR)/,,$<), creating $(subst ..,__,$@)"
	$(SILENT) $(MOC) $(MOC_FLAGS) -p $(subst ..,__,$(@D)) $< -o $(subst ..,__,$@)

.SECONDEXPANSION:
$(BINDIR)/%: $(SRCDIR)/Makefile
$(BINDIR)/%: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(BINDIR)
	$(SILENT) echo -e "$(INDENT_PRINT)--> Linking $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) mkdir -p $(@D)
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

$(LIBDIR)/%.so: $(SRCDIR)/Makefile
$(LIBDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(LIBDIR)
	$(SILENT) echo -e "$(INDENT_PRINT)--> Linking lib $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

$(PLUGINDIR)/%.so: $(SRCDIR)/Makefile
$(PLUGINDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(PLUGINDIR)
	$(SILENT) echo -e "$(INDENT_PRINT)--> Linking plugin $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)
