#*****************************************************************************
#               Makefile Build System for Fawkes: PDDL Parser
#                            -------------------
#   Created on Mon Nov 07 01:06:16 2011
#   Copyright (C) 2021 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
$(error config.mk must be included before pddl_parser.mk)
endif

ifneq ($(PKGCONFIG),)
  HAVE_PDDL_PARSER = $(if $(shell $(PKGCONFIG) --exists 'pddl_parser'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_PDDL_PARSER),1)
  CFLAGS_PDDL_PARSER = $(shell $(PKGCONFIG) --cflags 'pddl_parser')
  LDFLAGS_PDDL_PARSER = $(shell $(PKGCONFIG) --libs 'pddl_parser')
endif
