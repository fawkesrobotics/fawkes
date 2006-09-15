#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Wed Sep 06 02:39:46 2006
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

BASEDIR = .

SUBDIRS = src

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/rules.mk

.PHONY: apidoc
apidoc:
	$(SILENT) echo "--> Building documentation. This may take a while..."
	$(SILENT) rm -rf doc/api
	$(SILENT) $(DOXYGEN) doc/doxygen/api.doxygen
	$(SILENT) if [ "`wc -l warnings.txt | awk '{ print $$1 }'`" != "0" ]; then \
		$(RED); \
		echo "--> Warnings have been generated:"; \
		$(NORMAL); \
		cat warnings.txt; \
	else \
		$(GREEN); \
		echo "--> No warnings. Nice job."; \
		$(NORMAL); \
	fi
	$(SILENT) rm -rf warnings.txt

