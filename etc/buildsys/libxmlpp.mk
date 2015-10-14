#*****************************************************************************
#              Makefile Build System for Fawkes: libxml++ bits
#                            -------------------
#   Created on Wed Oct 14 11:53:00 2015
#   copyright (C) 2006-2015 by Tim Niemueller, KBSG
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
$(error config.mk must be included before libxmlpp.mk)
endif

ifndef __buildsys_libxmlpp_mk_
__buildsys_libxmlpp_mk_ := 1

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLPP    = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
  CFLAGS_LIBXMLPP  = -DHAVE_LIBXMLPP $(shell $(PKGCONFIG) --cflags libxml++-2.6)
  LDFLAGS_LIBXMLPP = $(shell $(PKGCONFIG) --libs libxml++-2.6)
  ifeq ($(HAVE_LIBXMLPP),1)
    HAVE_GLIBMM_246 = $(if $(shell $(PKGCONFIG) --atleast-version=2.46 'glibmm-2.4'; echo $${?/1/}),1,0)
    ifeq ($(HAVE_GLIBMM_246),1)
      CFLAGS_LIBXMLPP += $(CFLAGS_CPP11) -Wno-deprecated-declarations
    endif
  endif
endif

endif # __buildsys_libxmlpp_mk_
