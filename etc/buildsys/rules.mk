#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
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

# see http://make.paulandlesley.org/autodep.html
# see http://make.paulandlesley.org/rules.html

ifndef __buildsys_config_mk_
$(error config.mk must be included before rules.mk)
endif

ifndef __buildsys_rules_mk_
__buildsys_rules_mk_ := 1

include $(abspath $(BUILDSYSDIR)/ext/gmsl)

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

PARENTDIR=$(subst $(abspath $(TOP_BASEDIR))/,,$(SRCDIR)/)

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

ifeq ($(MAKECMDGOALS),clean)
  DISABLE_OBJS_all_WARNING = 1
  ifneq ($(wildcard $(SRCDIR)/tests),)
    ifneq ($(findstring tests,$(SUBDIRS)),tests)
      SUBDIRS += tests
    endif
  endif
endif

ifeq ($(findstring test,$(MAKECMDGOALS)),test)
  ifneq ($(wildcard $(SRCDIR)/tests),)
    ifneq ($(findstring tests,$(SUBDIRS)),tests)
      SUBDIRS += tests
    endif
  endif
endif

# If SOVER for lib was not set (SOVER_libname empty), set it to DEFAULT_SOVER
$(foreach L,$(LIBS_all:$(LIBDIR)/%.so=%) $(LIBS_gui:$(LIBDIR)/%.so=%) $(LIBS_test:$(LIBDIR)/%.so=%),$(if $(SOVER_$(subst /,_,$L)),,$(eval SOVER_$(subst /,_,$L) = $(DEFAULT_SOVER))))

ifdef __buildsys_lua_mk_
# Lua libraries do not set an SOVER, it's not checked anyway
$(foreach L,$(LIBS_all:$(LIBDIR)/lua/%.so=%),$(if $L,$(eval NOSOVER_lua_$(call nametr,$L)=1)))
endif

# Dependencies
-include $(DEPDIR)/*.d

# One to build 'em all
.PHONY: all gui test
ifeq ($(MAKELEVEL),1)
  EXTRA_ALL = $(LIBS_gui) $(PLUGINS_gui) $(BINS_gui) $(TARGETS_gui) $(MANPAGES_gui)
endif
all: presubdirs $(LIBS_all:%.so=%.$(SOEXT)) $(PLUGINS_all:%.so=%.$(SOEXT)) $(BINS_all) $(MANPAGES_all) $(TARGETS_all) $(EXTRA_ALL) subdirs | silent-nothing-to-do-all
gui: presubdirs $(LIBS_gui:%.so=%.$(SOEXT)) $(PLUGINS_gui:%.so=%.$(SOEXT)) $(BINS_gui) $(MANPAGES_gui) $(TARGETS_gui) subdirs | silent-nothing-to-do-gui
test: presubdirs $(LIBS_test:%.so=%.$(SOEXT)) $(PLUGINS_test:%.so=%.$(SOEXT)) $(BINS_test) $(TARGETS_test) exec_test subdirs | silent-nothing-to-do-test
uncolored-all: all
uncolored-gui: gui
uncolored-test: test

BUILT_PARTS=
.PHONY: silent-nothing-to-do-gui silent-nothing-to-do-all silent-nothing-to-do-test
silent-nothing-to-do-all:
	$(SILENTSYMB)if [ -z "$(BUILT_PARTS)" ]; then echo -e "$(INDENT_PRINT)--- Nothing to do in $(TBOLDGRAY)$(PARENTDIR)$(TNORMAL) for target$(if $(subst 1,,$(words $(MAKECMDGOALS))),s) $(TBOLDGRAY)$(MAKECMDGOALS)$(TNORMAL)"; fi
	$(eval BUILT_PARTS += $@)

silent-nothing-to-do-gui:
	$(SILENTSYMB)if [ -z "$(BUILT_PARTS)" ]; then echo -e "$(INDENT_PRINT)--- Nothing to do in $(TBOLDGRAY)$(PARENTDIR)$(TNORMAL) for target$(if $(subst 1,,$(words $(MAKECMDGOALS))),s) $(TBOLDGRAY)$(MAKECMDGOALS)$(TNORMAL)"; fi
	$(eval BUILT_PARTS += $@)

silent-nothing-to-do-test:
	$(SILENTSYMB)if [ -z "$(BUILT_PARTS)" ]; then echo -e "$(INDENT_PRINT)--- Nothing to do in $(TBOLDGRAY)$(PARENTDIR)$(TNORMAL) for target$(if $(subst 1,,$(words $(MAKECMDGOALS))),s) $(TBOLDGRAY)$(MAKECMDGOALS)$(TNORMAL)"; fi
	$(eval BUILT_PARTS += $@)

.PHONY: clean
clean: presubdirs subdirs
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--> Cleaning up directory $(TBOLDGRAY)$(PARENTDIR)$(TNORMAL)"
	$(SILENT) if [ "$(SRCDIR)/$(OBJDIR)" != "/" ]; then rm -rf "$(SRCDIR)/$(OBJDIR)" ; fi
	$(SILENT) if [ -n "$(DEPDIR)" ]; then rm -rf "$(DEPDIR)" ; fi
	$(SILENT)$(foreach B,$(BINS_all),rm -f $(B);)
	$(SILENT)$(foreach L,$(LIBS_all:%.so=%.$(SOEXT)),rm -f $(addsuffix *,$(L));)
	$(SILENT)$(foreach P,$(PLUGINS_all:%.so=%.$(SOEXT)),rm -f $(P);)
	$(SILENT)$(foreach M,$(MANPAGES_all),rm -f $(M);)
	$(SILENT)$(foreach T,$(TARGETS_all),rm -rf $(T);)
	$(SILENT)$(foreach B,$(BINS_gui),rm -f $(B);)
	$(SILENT)$(foreach L,$(LIBS_gui:%.so=%.$(SOEXT)),rm -f $(addsuffix *,$(L));)
	$(SILENT)$(foreach P,$(PLUGINS_gui:%.so=%.$(SOEXT)),rm -f $(P);)
	$(SILENT)$(foreach M,$(MANPAGES_gui),rm -f $(M);)
	$(SILENT)$(foreach T,$(TARGETS_gui),rm -rf $(T);)
	$(SILENT)$(foreach B,$(BINS_test),rm -f $(B);)
	$(SILENT)$(foreach L,$(LIBS_test:%.so=%.$(SOEXT)),rm -f $(addsuffix *,$(L));)
	$(SILENT)$(foreach P,$(PLUGINS_test:%.so=%.$(SOEXT)),rm -f $(P);)
	$(SILENT)$(foreach M,$(MANPAGES_test),rm -f $(M);)
	$(SILENT)$(foreach T,$(TARGETS_test),rm -rf $(T);)
	$(SILENT)$(foreach E,$(CLEAN_FILES),rm -rf $(E);)

.PHONY: presubdirs $(PRESUBDIRS) subdirs $(SUBDIRS)
presubdirs: $(PRESUBDIRS)
subdirs: $(SUBDIRS)

SUBDIRS_DEPS = $(foreach goal,$(filter-out clean,$(MAKECMDGOALS)), \
	$(LIBS_$(goal)) $(PLUGINS_$(goal)) $(BINS_$(goal)) $(MANPAGES_$(goal)) $(TARGETS_$(goal)))
ifeq ($(findstring all,$(MAKECMDGOALS)),all)
  SUBDIRS_DEPS += $(EXTRA_ALL)
endif

ifneq ($(strip $(SUBDIRS_DEPS)),)
subdirs: | $(SUBDIRS_DEPS)
$(SUBDIRS_DEPS): | presubdirs
endif

# Either presubdirs *or* subdirs have been specified
ifneq ($(PRESUBDIRS)$(SUBDIRS),)

  ifneq ($(SUBDIRS),)
    ifneq ($(PRESUBDIRS),)
      # Both, subdirs *and* presubdirs have been specified
$(SUBDIRS): | $(PRESUBDIRS)
    endif
  endif

$(PRESUBDIRS) $(SUBDIRS):
	$(eval BUILT_PARTS += $@)
	$(SILENTSYMB) if [ ! -d "$(abspath $(SRCDIR)/$(@))" ]; then \
		echo -e "$(INDENT_PRINT)---$(TRED)Directory $(TNORMAL)$(TBOLDRED)$@$(TNORMAL)$(TRED) does not exist, check [PRE]SUBDIRS variable$(TNORMAL)"; \
		exit 1; \
	else \
		echo -e "$(INDENT_PRINT)--> Entering sub-directory $(TBOLDGRAY)$(PARENTDIR)$(TNORMAL)$(TBOLDBLUE)$@$(TNORMAL)"; \
		$(MAKE) --no-print-directory --no-keep-going -C "$(abspath $(SRCDIR)/$@)" \
		$(MAKECMDGOALS) INDENT="$(INDENT)$(INDENT_STRING)" \
		SRCDIR="$(abspath $(SRCDIR)/$@)" OBJSSUBMAKE=0 || exit $$?; \
		if [ "$(MAKECMDGOALS)" != "clean" ]; then \
			echo -e "$(INDENT_PRINT)$(subst -, ,$(INDENT_STRING))<-- Leaving $(PARENTDIR)$(TBOLDGRAY)$@$(TNORMAL)"; \
		fi \
	fi
endif


ifdef OBJS_all
  ifneq ($(OBJS_all),)
    # Do not delete .o files to allow for incremental builds
    .SECONDARY: $(OBJS_all)
    # Whenever the Makefile is modified rebuild everything
    $(OBJS_all): $(SRCDIR)/Makefile
    # processs presubdirs before OBJS_all
    ifneq ($(PRESUBDIRS),)
      $(OBJS_all): | presubdirs
    endif
  endif
else
  ifneq ($(LIBS_all)$(PLUGINS_all)$(BINS_all)$(LIBS_gui)$(PLUGINS_gui)$(BINS_gui)$(LIBS_test)$(PLUGINS_test)$(BINS_test),)
    ifneq ($(DISABLE_OBJS_all_WARNING),1)
      $(warning OBJS_all is not set. This is probably a bug. If you intended this set DISABLE_OBJS_all_WARNING to 1 to get rid of this warning.)
    endif
  endif
endif

# Cancel built-in implicit rules, they suck
%: %.cpp
%: %.c

# NOTE: .. are replaced by __. This is needed to have objects that are in upper relative
# directories are build in .objs, another change is needed below in bin, lib and plugin targets,
# mocs etc.
%.o: %.cpp
	$(eval BUILT_PARTS += $@)
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[C++] $(PARENTDIR)$(TBOLDGRAY)$(subst $(SRCDIR)/,,$<)$(TNORMAL)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -MD -MF $(DEPFILE).td $(CFLAGS_BASE) $(if $(CFLAGS_$(subst /,_,$*)),$(CFLAGS_$(subst /,_,$*)),$(CFLAGS))  \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT)sed -e '/^[^:]*\//! s/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(DEPFILE).td > $(DEPFILE).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEPFILE).td >> $(DEPFILE).d; \
	rm -f $(DEPFILE).td

%.o: %.c
	$(SILENT) mkdir -p $(DEPDIR)
	$(SILENT) mkdir -p $(@D)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[ C ] $(PARENTDIR)$(TBOLDGRAY)$(subst $(SRCDIR)/,,$<)$(TNORMAL)"
	$(SILENT) mkdir -p $(dir $(subst ..,__,$@))
	$(SILENT) $(CC) -MD -MF $(DEPFILE).td $(CFLAGS_BASE) $(if $(CFLAGS_$*),$(CFLAGS_$*),$(CFLAGS)) \
	$(addprefix -I,$(INCS_$*)) $(addprefix -I,$(INCDIRS)) -c -o $(subst ..,__,$@) $<
	$(SILENT)sed -e '/^[^:]*\//! s/^\([^:]\+\): \(.*\)$$/$(subst /,\/,$(@D))\/\1: \2/' < $(DEPFILE).td > $(DEPFILE).d; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e 's/^ *//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEPFILE).td >> $(DEPFILE).d; \
	rm -f $(DEPFILE).td

moc_%.cpp: %.h
	$(eval BUILT_PARTS += $@)
	$(SILENTSYMB) echo "$(INDENT_PRINT)--- Running Qt moc on $(subst $(SRCDIR)/,,$<), creating $(subst ..,__,$@)"
	$(SILENT) $(MOC) $(MOC_FLAGS) -p "../$(subst ..,__,$(@D))" $< -o $(subst ..,__,$@)

$(foreach MS,$(MANPAGE_SECTIONS),$(MANDIR)/man$(MS)/%.$(MS)): %.txt
	$(SILENT) mkdir -p $(@D)
	$(SILENT)if type -P $(ASCIIDOC_A2X) >/dev/null 2>&1; then \
	echo -e "$(INDENT_PRINT)[MAN] $(PARENTDIR): $(TBOLDGREEN)$(subst $(abspath $(TOP_BASEDIR))/,,$(abspath $(dir $@)))/$*$(TNORMAL)"; \
	TEMPFILE=$$(mktemp -t fawkes_manpage_$*_XXXXXXXXXX); \
	$(ASCIIDOC_A2X) -f manpage \
	--asciidoc-opts='-f $(BASEDIR)/doc/asciidoc.conf -afawkes_version="$(FAWKES_VERSION)"' \
	-D $(@D) $< >$$TEMPFILE 2>&1; \
	if egrep -v '^(Note: Writing $(@F)|Writing $(@F) for refentry)$$' $$TEMPFILE >/dev/null 2>&1; then \
		cat $$TEMPFILE; \
	fi; \
	rm $$TEMPFILE; \
	rm -f $(SRCDIR)/$*.xml; \
	else \
		echo -e "$(INDENT_PRINT)=== $(TYELLOW)Cannot generate man page for $* (asciidoc not installed)$(TNORMAL)"; \
	fi

# execute every test in $(BINS_test)
exec_test: $(patsubst $(BINDIR)/%,exec_%,$(BINS_test))

# execution of a single test.
exec_gtest_%: $(BINDIR)/gtest_%
	$(eval BUILT_PARTS += $@)
	$(SILENT)echo -e "$(INDENT_PRINT)[TEST] $(BINDIR)/gtest_$*"
	$(SILENT)exec $(BINDIR)/gtest_$* --gtest_color=yes | sed 's/^/$(INDENT_PRINT)[TEST] /'; \
		test $${PIPESTATUS[0]} -eq 0

exec_gdb_gtest_%: $(BINDIR)/gtest_%
	$(eval BUILT_PARTS += $@)
	$(SILENT)echo -e "$(INDENT_PRINT)[TEST-GDB] $(BINDIR)/gtest_$*"
	$(SILENT)exec gdb $(BINDIR)/gtest_$*

.SECONDEXPANSION:
$(BINDIR)/%: $$(OBJS_$$(call nametr,$$*))
	$(eval BUILT_PARTS += $@)
	$(SILENT) mkdir -p $(@D)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[BIN] $(PARENTDIR): $(TBOLDGREEN)$(subst $(abspath $(TOP_BASEDIR))/,,$(abspath $(BINDIR)))/$*$(TNORMAL)"
	$(SILENT) $(if $(LD_$(call nametr,$*)),$(LD_$(call nametr,$*)),$(LD)) \
	-o $@ $(subst ..,__,$^) $(LDFLAGS_BASE) \
	$(if $(call seq,$(origin LDFLAGS_$(call nametr,$*)),undefined),$(LDFLAGS),$(LDFLAGS_$(call nametr,$*))) \
	$(addprefix -l,$(LIBS_$(call nametr,$*))) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$(call nametr,$*))) $(addprefix -L,$(LIBDIRS))
ifeq ($(WARN_MISSING_MANPAGE),1)
	$(if $(strip $(foreach S,$(MANPAGE_SECTIONS),$(filter $(MANDIR)/man$S/$*.$S,$(MANPAGES_all) $(MANPAGES_gui)))),,$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- $(TYELLOW)Warning:$(TNORMAL) $(PARENTDIR)$(TBOLDGRAY)$*$(TNORMAL) does not have a man page$(TNORMAL)")
endif

$(LIBDIR)/%.so: $$(OBJS_$$(call nametr,$$*))
	$(eval BUILT_PARTS += $@)
	$(SILENT) mkdir -p $(@D)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[LIB] $(PARENTDIR): $(TBOLDGREEN)$(subst $(abspath $(TOP_BASEDIR))/,,$(abspath $(LIBDIR)))/$*$(TNORMAL)"
	$(SILENT) $(if $(LD_$(call nametr,$*)),$(LD_$(call nametr,$*)),$(LD)) \
	-o $@$(if $(NOSOVER_$(call nametr,$*)),,.$(SOVER_$(call nametr,$*))) $(subst ..,__,$^) \
	$(if $(NOSOVER_$(call nametr,$*)),,-Wl,-soname=$(@F).$(SOVER_$(call nametr,$*))) \
	$(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS) $(LDFLAGS_$(call nametr,$*)) \
	$(if $(ALLOW_UNDEF_$(call nametr,$*)),,$(LDFLAGS_NO_UNDEF)) \
	$(addprefix -l,$(LIBS_$(call nametr,$*))) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$(call nametr,$*))) $(addprefix -L,$(LIBDIRS))
	$(if $(NOSOVER_$(call nametr,$*)),, \
	$(SILENT) ln -fs $(@F).$(SOVER_$(call nametr,$*)) $@; \
	ln -fs $(@F).$(SOVER_$(call nametr,$*)) $@.$(firstword $(call split,.,$(SOVER_$(call nametr,$*)))); \
	)

### Check if there are special additions
ifneq ($(wildcard $(BUILDSYSDIR)/btypes/rules_$(BUILD_TYPE).mk),)
  include $(BUILDSYSDIR)/btypes/rules_$(BUILD_TYPE).mk
else
  ifneq ($(SECONDARY_BUILDSYSDIR),)
    ifneq ($(wildcard $(SECONDARY_BUILDSYSDIR)/btypes/rules_$(BUILD_TYPE).mk),)
      include $(SECONDARY_BUILDSYSDIR)/btypes/rules_$(BUILD_TYPE).mk
    endif
  endif
endif

endif # __buildsys_rules_mk_

