#*****************************************************************************
#     Makefile Build System for Fawkes: Config Settings specific to Fawkes
#                            -------------------
#   Created on Tue Oct 30 14:40:55 2007
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

LIBS_all:=$(LIBS_all:%.so=%.dylib)
LIBS_gui:=$(LIBS_gui:%.so=%.dylib)

# Remove old rule
#$(LIBDIR)/%.so: $$(OBJS_$$(subst /,_,$$*))
## Redirect do .dylib
#$(LIBDIR)/%.so: $(LIBDIR)/%.dylib
#	$(SILENT)ln -s $< $@

$(LIBDIR)/%.dylib: $$(OBJS_$$(subst /,_,$$*))
	$(SILENT) mkdir -p $(@D)
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)=== Linking lib $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(if $(LD_$(call nametr,$*)),$(LD_$(call nametr,$*)),$(LD)) -o $@ $(subst ..,__,$^) \
	$(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS) $(LDFLAGS_$(subst /,_,$*)) \
	$(addprefix -l,$(LIBS_$(subst /,_,$*))) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(wildcard $(LIBDIRS_$(subst /,_,$*)))) $(addprefix -L,$(wildcard $(LIBDIRS)))

# Rule for building plugins in PLUGINDIR
$(PLUGINDIR)/%.dylib: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking plugin $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(if $(LD_$(call nametr,$*)),$(LD_$(call nametr,$*)),$(LD)) -o $@ $(subst ..,__,$^) \
	$(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS) $(LDFLAGS_$(subst /,_,$*)) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS))
	$(SILENT)$(NM) $@ | grep -q plugin_factory; \
	if [ "$$?" != "0" ]; then \
		rm $@; \
		echo -e "$(INDENT_PRINT)--- $(TRED)Plugin has no plugin_factory symbol. Forgot EXPORT_PLUGIN?$(TNORMAL)"; \
		exit 3; \
	fi
	$(SILENT)$(NM) $@ | grep -q plugin_description; \
	if [ "$$?" != "0" ]; then \
		rm $@; \
		echo -e "$(INDENT_PRINT)--- $(TRED)Plugin has no plugin_description symbol. Forgot PLUGIN_DESCRIPTION?$(TNORMAL)"; \
		exit 3; \
	fi

