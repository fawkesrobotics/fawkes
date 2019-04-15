#*****************************************************************************
#       Makefile Build System for Fawkes: Force "make check" before push
#                            -------------------
#   Created on Mon Apr 08 13:41:08 2019 +0200
#   Copyright (C) 2006-2019 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before check.mk)
endif

$(TOP_BASEDIR)/.git/hooks/pre-push: $(FAWKES_BASEDIR)/etc/git-hooks/pre-push
	$(SILENT)mkdir -p $(TOP_BASEDIR)/.git/hooks
	$(SILENTSYMB) echo -e "$(INDENT_PRINT)[GIT] installing pre-push hook $@"
	$(SILENT)install -m 0755 $< $@

all: $(TOP_BASEDIR)/.git/hooks/pre-push
