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

include $(BUILDSYSDIR)/btypes/rules_fawkes.mk

.PHONY: deploy
deploy:
	$(SILENT) echo -e "$(INDENT_PRINT)=== Deploying to $(TBOLDBLUE)$(HOST)$(TNORMAL) ---"
	$(SILENT) echo -e "$(INDENT_PRINT)--- Copying Fawkes items"
	$(SILENT) RSYNC_DONE=1; while [ "$$RSYNC_DONE" != "0" ]; do \
		$(RSYNC) -ak $(RSYNC_FLAGS) $(RSYNC_DIRS) $(RSYNC_USER)@$(HOST):$(TARGET_BASEDIR)/ ; \
		RSYNC_DONE=$$?; \
		if [ "$$RSYNC_DONE" == "20" ]; then \
			echo -e "$(TRED)Interrupted.$(TNORMAL)"; \
			exit $$RSYNC_DONE; \
		fi; \
		if [ "$$RSYNC_DONE" != "0" ]; then \
			echo -e "$(INDENT_PRINT)--- $(TYELLOW)Transmission failed, retrying$(TNORMAL)"; \
		fi; \
	done
	$(SILENT) echo -e "$(INDENT_PRINT)--- Copying NaoQi Modules"
	$(SILENT) RSYNC_DONE=1; while [ "$$RSYNC_DONE" != "0" ]; do \
		$(RSYNC) -ak $(RSYNC_NAOQIMODS_FLAGS) $(RSYNC_NAOQIMODS) $(RSYNC_USER)@$(HOST):$(TARGET_NAOQIROOT)/lib/naoqi/ ; \
		RSYNC_DONE=$$?; \
		if [ "$$RSYNC_DONE" == "20" ]; then \
			echo -e "$(TRED)Interrupted.$(TNORMAL)"; \
			exit $$RSYNC_DONE; \
		fi; \
		if [ "$$RSYNC_DONE" != "0" ]; then \
			echo -e "$(INDENT_PRINT)--- $(TYELLOW)Transmission failed, retrying$(TNORMAL)"; \
		fi; \
	done

