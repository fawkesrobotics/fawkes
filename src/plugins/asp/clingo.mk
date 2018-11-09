#*****************************************************************************
#              Makefile Build System for Fawkes: Clingo bits
#                            -------------------
#   Created on Fri Nov 09 12:44:24 2018 +0100
#   Copyright (C) 2016 Björn Schäpers
#                 2018 Tim Niemueller [www.niemueller.org]
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifneq ($(CLINGO_DIR),)
  ifneq ($(wildcard $(CLINGO_DIR)/build/bin/libclingo.so),)
    ifneq ($(wildcard $(CLINGO_DIR)/libclingo/clingo/clingocontrol.hh),)
      HAVE_CLINGO=1
		else
			CLINGO_ERROR=Clingo not found
    endif
	else
		CLINGO_ERROR=Clingo lib not found
  endif
else
  CLINGO_ERROR=CLINGO_DIR not set
endif

ifeq ($(HAVE_CLINGO),1)
	CFLAGS_CLINGO = -DWITH_THREADS=1 \
		-I $(CLINGO_DIR)/libclasp \
		-I $(CLINGO_DIR)/libclingo \
		-I $(CLINGO_DIR)/libgringo \
		-I $(CLINGO_DIR)/liblp \
		-I $(CLINGO_DIR)/libprogram_opts
	LDFLAGS_CLINGO = -L$(CLINGO_DIR)/build/bin/ -Wl,-rpath=$(CLINGO_DIR)/build/bin/ -lclingo
endif

