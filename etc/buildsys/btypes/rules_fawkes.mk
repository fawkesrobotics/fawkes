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

# Rule for building plugins in PLUGINDIR
$(PLUGINDIR)/%.so: $$(OBJS_$$(call nametr,$$*))
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking plugin $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(if $(LD_$(call nametr,$*)),$(LD_$(call nametr,$*)),$(LD)) \
	-o $@ $(subst ..,__,$^) \
	$(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS) $(LDFLAGS_$(call nametr,$*)) \
	$(addprefix -l,$(LIBS_$(call nametr,$*))) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$(call nametr,$*))) $(addprefix -L,$(LIBDIRS))
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

