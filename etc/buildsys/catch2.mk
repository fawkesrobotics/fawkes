#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Mon 10 Feb 2020 13:50:59 CET
#   Copyright (C) 2020 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
$(error config.mk must be included before catch.mk)
endif

ifndef __buildsys_catch2_mk_
__buildsys_catch2_mk_ := 1

ifneq ($(PKGCONFIG),1)
  HAVE_CATCH2 = $(if $(shell $(PKGCONFIG) --exists 'catch2'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_CATCH2),1)
  CFLAGS_CATCH2 = $(shell $(PKGCONFIG) --cflags 'catch2')
  LDFLAGS_CATCH2 = $(shell $(PKGCONFIG) --libs 'catch2')
endif

endif # __buildsys_catch2_mk_
