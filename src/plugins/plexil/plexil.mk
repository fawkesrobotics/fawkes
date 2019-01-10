#*****************************************************************************
#                Makefile Build System for Fawkes: PLEXIL bits
#                            -------------------
#   Created on Mon Aug 13 11:28:55 2018
#   Copyright (C) 2011-2018 by Tim Niemueller
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
$(error config.mk must be included before clips.mk)
endif

include $(BUILDSYSDIR)/ext/gmsl

_CFLAGS_PLEXIL_COMMON = -DHAVE_PLEXIL -Wno-deprecated

ifneq ($(PLEXIL_HOME),)
  ifneq ($(wildcard $(PLEXIL_HOME)/include/ExecApplication.hh),)
    ifneq ($(wildcard $(PLEXIL_HOME)/lib/libPlexilAppFramework.so),)
      HAVE_PLEXIL=1
      CFLAGS_PLEXIL  += $(_CFLAGS_PLEXIL_COMMON) -I$(PLEXIL_HOME)/include
      LDFLAGS_PLEXIL += -L$(PLEXIL_HOME)/lib
    else
      PLEXIL_ERRORS += PLEXIL_app_framework_lib_not_found
    endif
  else
    PLEXIL_ERRORS += PLEXIL_headers_not_found
  endif
else
  PLEXIL_ERRORS += PLEXIL__HOME_not_set
endif

ifneq ($(HAVE_PLEXIL),1)
  ifneq ($(PKGCONFIG),)
    HAVE_PLEXIL = $(if $(shell $(PKGCONFIG) --exists 'plexil'; echo $${?/1/}),1,0)
    ifeq ($(HAVE_PLEXIL),1)
      CFLAGS_PLEXIL  = $(_CFLAGS_PLEXIL_COMMON) $(CFLAGS_CPP14) \
                       $(shell $(PKGCONFIG) --cflags 'plexil')
      LDFLAGS_PLEXIL = $(shell $(PKGCONFIG) --libs 'plexil')
    else
      PLEXIL_ERRORS += PLEXIL_not_found_via_pkg-config
    endif
  endif
endif

ifneq ($(HAVE_PLEXIL),1)
  ifneq ($(wildcard /usr/include/plexil/ExecApplication.hh),)
    ifneq ($(wildcard /usr/lib/libPlexilAppFramework.so /usr/lib64/libPlexilAppFramework.so /usr/lib32/libPlexilAppFramework.so),)
      HAVE_PLEXIL=1
      CFLAGS_PLEXIL  += $(_CFLAGS_PLEXIL_COMMON) -I/usr/include/plexil
      LDFLAGS_PLEXIL =
    else
      PLEXIL_ERRORS += PLEXIL_not_found_as_system_wide_installation
    endif
  endif
endif

ifneq ($(HAVE_PLEXIL),1)
  _COMMA := ,
  PLEXIL_ERROR=$(subst --X--,_,$(subst _, ,$(subst __,--X--,$(subst :,$(_COMMA) ,$(call merge,:,$(PLEXIL_ERRORS))))))
endif
