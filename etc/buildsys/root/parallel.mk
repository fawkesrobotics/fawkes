#*****************************************************************************
#            Makefile Build System for Fawkes: parallel-* targets
#                              -------------------
#   Created on Wed May 30 11:59:55 2018
#   Copyright (C) 2006-2018 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

# Parallel implicit targets
parallel-%:
	$(SILENT)if [ "$(filter -j% --jobs=% --jobserver-auth=%,$(MAKEFLAGS))" != "" ]; then \
		echo -e "$(INDENT_STRING)[ERROR] $(TRED)Cannot call target $@ with '$(filter -j% --jobs=%,$(MAKEFLAGS))'$(TNORMAL)"; \
		exit 1; \
	else \
		$(MAKE) --no-print-directory --no-keep-going -j$(shell nproc) $*; \
	fi
