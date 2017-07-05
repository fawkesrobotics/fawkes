#*****************************************************************************
#                 Makefile Build System for Fawkes: Ctemplate
#                            -------------------
#   Created on Fri Oct 13 19:32:21 2016
#   Author  Frederik Zwilling
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
$(error config.mk must be included before ctemplate.mk)
endif

ifndef __buildsys_ctemplate_mk_
__buildsys_ctemplate_mk_ := 1


ifneq ($(PKGCONFIG),)
  HAVE_CTEMPLATE   = $(if $(shell $(PKGCONFIG) --exists 'libctemplate'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_CTEMPLATE),1)
  CFLAGS_CTEMPLATE  = $(shell $(PKGCONFIG) --cflags 'libctemplate')
  LDFLAGS_CTEMPLATE = $(shell $(PKGCONFIG) --libs 'libctemplate')
endif

endif # __buildsys_ctemplate_mk_