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

-include $(DEPDIR)/*.d

.SECONDARY: $(OBJS_all)

.PHONY: all
all: subdirs $(LIBS_all) $(PLUGINS_all) $(BINS_all)

.PHONY: clean
clean: subdirs
	$(SILENT) echo "$(INDENT)--> Cleaning up directory $(CURDIR)"
	$(SILENT) if [ "$(SRCDIR)/$(OBJDIR)" != "/" ]; then rm -rf $(SRCDIR)/$(OBJDIR) ; fi
	$(SILENT) if [ "$(DEPDIR)" != "" ]; then rm -rf $(DEPDIR) ; fi
	$(SILENT) if [ "$(BINS_all)" != "" ]; then rm -rf $(BINS_all) ; fi
	$(SILENT) if [ "$(LIBS_all)" != "" ]; then rm -rf $(LIBS_all) ; fi
	$(SILENT) if [ "$(PLUGINS_all)" != "" ]; then rm -rf $(PLUGINS_all) ; fi

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)

ifneq ($(SUBDIRS),)
$(SUBDIRS):
	$(SILENT) echo "$(INDENT)--- Entering sub-directory $@ ---"
	$(SILENT) $(MAKE) --no-print-directory -C $@ $(MAKECMDGOALS) INDENT="$(INDENT)$(INDENT_STRING)"
	$(SILENT) echo "$(INDENT)--- Leaving sub-directory $@ ---"
endif

# NOTE: .. are replaced by __. This is needed to have objects that are in upper relative
# directories are build in .objs, another change is needed below in bin, lib and plugin targets,
# mocs etc.
%.o: %.cpp
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo "$(INDENT)--> Compiling $(<F) (C++)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -Wp,-M,-MF,$(df).d $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT) cp -f $(df).d $(df).td; \
	sed -e 's/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(df).td > $(df).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(df).td >> $(df).d; \
	rm -f $(df).td

moc_%.cpp: %.h
	$(SILENT) echo "$(INDENT)--- Running Qt moc on $<, creating $(subst ..,__,$@)"
	$(SILENT) $(MOC) $(MOC_FLAGS) -p $(subst ..,__,$(@D)) $< -o $(subst ..,__,$@)

.SECONDEXPANSION:
$(BINDIR)/%: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(BINDIR)
	$(SILENT) echo "$(INDENT)--> Linking $* ---"
	$(SILENT) mkdir -p $(@D)
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

$(LIBDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(LIBDIR)
	$(SILENT) echo "$(INDENT)--> Linking lib $* ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

$(PLUGINDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(PLUGINDIR)
	$(SILENT) echo "$(INDENT)--> Linking plugin $* ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $(subst ..,__,$^)

