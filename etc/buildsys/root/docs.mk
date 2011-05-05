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
$(error config.mk must be included before docs.mk)
endif

ifndef __buildsys_root_docs_mk_
__buildsys_root_docs_mk_ := 1

.PHONY: apidoc quickdoc tracdoc
apidoc: api.doxygen
quickdoc: api-quick.doxygen
tracdoc: api-trac.doxygen

%.doxygen:
	$(SILENT) echo "--> Building documentation ($@). This may take a while..."
	$(SILENT) rm -rf doc/api
	$(SILENT) mkdir -p doc/api
	$(SILENT) $(DOXYGEN) $(DOCDIR)/doxygen/$*$(if $(SUBMODULE_EXTERN),-submodule).doxygen >/dev/null 2>&1
	$(SILENT) if [ "`wc -l warnings.txt | awk '{ print $$1 }'`" != "0" ]; then \
		echo -e "$(TRED)--> Warnings have been generated:$(TNORMAL)"; \
		cat warnings.txt; \
		exit 1; \
	else \
		echo -e "$(TGREEN)--> No warnings. Nice job.$(TNORMAL)"; \
	fi

endif # __buildsys_root_docs_mk_

