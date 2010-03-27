#*****************************************************************************
#              Makefile Build System for Fawkes: Download Support
#                            -------------------
#   Created on Sat Apr 05 00:11:59 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before download.mk)
endif

ifndef __buildsys_download_mk_
__buildsys_download_mk := 1

DOWNLOAD_APPS=curl wget fetch
DOWNLOAD_APP=$(firstword $(foreach a,$(DOWNLOAD_APPS),$(shell which $a 2>/dev/null)))

DOWNLOAD_APP_PARAMS_curl=-O

define download-files
if [ -z "$(DOWNLOAD_APP)" ]; then \
	echo -e "$(INDENT_PRINT)--> $(TRED)Cannot download files$(TNORMAL) (no download application found)"; \
else \
	if [ -z "$(URLS_$(patsubst download-%,%,$@))" ]; then \
		echo -e "$(INDENT_PRINT)--> $(TRED)Cannot download files$(TNORMAL) (no URLs given)"; \
	else \
		for u in $(URLS_$(patsubst download-%,%,$@)); do \
			echo -e "$(INDENT_PRINT)--> Downloading $$u using $(notdir $(DOWNLOAD_APP))"; \
			if ! $(DOWNLOAD_APP) $(DOWNLOAD_APP_PARAMS_$(notdir $(DOWNLOAD_APP))) "$$u"; then \
				echo -e "$(INDENT_PRINT)--> $(TRED)Download of $$u failed$(TNORMAL)"; \
				exit 4; \
			fi \
		done \
	fi \
fi
endef

endif # __buildsys_download_mk_

