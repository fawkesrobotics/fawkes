#*****************************************************************************
#     Makefile Build System for Fawkes: Config Settings specific to Fawkes
#                            -------------------
#   Created on Tue Oct 30 14:40:55 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
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
$(PLUGINDIR)/%.so: $$(OBJS_$$*)
	$(SILENT) mkdir -p $(@D)
	$(SILENT) echo -e "$(INDENT_PRINT)=== Linking plugin $(TBOLDGREEN)$*$(TNORMAL) ---"
	$(SILENT) $(CC) -o $@ $(subst ..,__,$^) \
	$(LDFLAGS_BASE) $(LDFLAGS_SHARED) $(LDFLAGS_LIBDIRS) $(LDFLAGS) $(LDFLAGS_$*) \
	$(addprefix -l,$(LIBS_$*)) $(addprefix -l,$(LIBS)) \
	$(addprefix -L,$(LIBDIRS_$*)) $(addprefix -L,$(LIBDIRS))

