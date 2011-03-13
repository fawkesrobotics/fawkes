#*****************************************************************************
#     Makefile Build System for Fawkes: Documentation config and targets
#                            -------------------
#   Created on Wed Mar 31 14:31:57 2010
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

ifndef __buildsys_config_mk_
$(error config.mk must be included before btmgmt.mk)
endif

ifndef __buildsys_root_btmgmt_mk_
__buildsys_root_btmgmt_mk_ := 1

.PHONY: switch-buildtype print-buildtype
switch-buildtype:
	$(SILENT) if [ -z "$(BT)" ]; then \
		echo -e "$(INDENT_PRINT)$(TRED)--- Usage: make switch-buildtype BT=new_buildtype$(TNORMAL)"; \
		echo -e "$(INDENT_PRINT)$(TRED)---    or: make switch-buildtype-new_buildtype$(TNORMAL)"; \
	elif [ "$(BUILD_TYPE)" = "$(BT)" ]; then \
		echo -e "$(INDENT_PRINT)$(TYELLOW)--- Build type $(BT) is already set$(TNORMAL)"; \
	else \
		echo -e "$(INDENT_PRINT)--- Switching build type from $(BUILD_TYPE) to $(BT)"; \
		if [ -e "$(TOP_BASEDIR)/etc/buildsys/buildtype.mk" ]; then \
			sed -i.bak -e 's/^BUILD_TYPE=.*$$/BUILD_TYPE=$(BT)/' $(TOP_BASEDIR)/etc/buildsys/buildtype.mk; \
			rm -f $(TOP_BASEDIR)/etc/buildsys/buildtype.mk.bak; \
		else \
			mkdir -p $(TOP_BASEDIR)/etc/buildsys; \
			echo "BUILD_TYPE=$(BT)" > $(TOP_BASEDIR)/etc/buildsys/buildtype.mk; \
		fi; \
		for D in $(BINDIR) $(LIBDIR) $(PLUGINDIR); do \
			rm -rf $${D}_$(BUILD_TYPE); \
			mv $$D $${D}_$(BUILD_TYPE); \
			if [ -d $${D}_$(BT) ]; then \
				mv $${D}_$(BT) $$D; \
			else \
				mkdir $$D; \
			fi; \
		done; \
	fi

# Easier to remember target than having to remember BT variable
switch-buildtype-%:
	$(SILENT)$(MAKE) --no-print-directory switch-buildtype BT=$* 

print-buildtype:
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)--- Current build type: $(BUILD_TYPE)";

endif # __buildsys_root_btmgmt_mk_

