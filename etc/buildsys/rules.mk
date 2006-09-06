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
all: $(LIBS_all) $(PLUGINS_all) $(BINS_all)

.PHONY: clean
clean:
	$(SILENT) echo "--> Cleaning up directory"
	$(SILENT) rm -rf $(SRCDIR)/$(OBJDIR)
	$(SILENT) rm -rf $(DEPDIR)
	$(SILENT) rm -rf $(BINS_all)
	$(SILENT) rm -rf $(LIBS_all)
	$(SILENT) rm -rf $(PLUGINS_all)

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)

ifneq ($(SUBDIRS),)
$(SUBDIRS):
	$(SILENT) echo "--> Entering sub-directory $@ ---"
	$(SILENT) $(MAKE) --no-print-directory -C $@
endif

%.o: %.cpp
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo "--> Compiling $(<F) (C++)"
	$(SILENT) $(CC) -Wp,-M,-MF,$(df).d $(CFLAGS_BASE) $(CFLAGS) $(CFLAGS_$*) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $@ $<
	$(SILENT) cp -f $(df).d $(df).td; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(df).td >> $(df).d; \
	rm -f $(df).td

moc_%.cpp: %.h
	$(SILENT) echo "--- Running Qt moc on $<, creating $@"
	$(SILENT) $(MOC) $(MOC_FLAGS) -p $(@D) $< -o $@

.SECONDEXPANSION:
$(BINDIR)/%: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(BINDIR)
	$(SILENT) echo "--> Linking $* ---"
	$(SILENT) mkdir -p $(@D)
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $^

$(LIBDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(LIBDIR)
	$(SILENT) echo "--> Linking lib $* ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $^

$(PLUGINDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(PLUGINDIR)
	$(SILENT) echo "--> Linking plugin $* ---"
	$(SILENT) $(CC) $(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS)) \
	-o $@ $^

