#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Fri 13 Oct 2017 13:55:21 CEST
#   Copyright (C) 2017 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/boost.mk

ifeq ($(HAVE_BOOST),1)
  HAVE_PDDL_PARSER = 1
else
  HAVE_PDDL_PARSER = 0
endif
