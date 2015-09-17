#*****************************************************************************
#                Makefile Build System for Fawkes: CLIPS bits
#                            -------------------
#   Created on Sat Jun 16 14:49:41 2012 (Mexico City)
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifndef __buildsys_clips_mk_
__buildsys_clips_mk_ := 1

# We use range-for loops
CLIPS_GCC_MINV_MAJ = 4
CLIPS_GCC_MINV_MIN = 6

CLIPS_ERROR=

ifneq ($(PKGCONFIG),)
  HAVE_CLIPS = $(if $(shell $(PKGCONFIG) --exists 'clipsmm-1.0'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_CLIPS),1)
    ifeq ($(CC),gcc)
      ifneq ($(call gcc_atleast_version,$(CLIPS_GCC_MINV_MAJ),$(CLIPS_GCC_MINV_MIN)),1)
        HAVE_CLIPS=
	CLIPS_ERROR = GCC version too old, have $(GCC_VERSION), required $(CLIPS_GCC_MINV_MAJ).$(CLIPS_GCC_MINV_MIN)
      endif
    endif
  else
    CLIPS_ERROR = CLIPS not found
  endif
else
  CLIPS_ERROR = pkg-config not available
endif

ifeq ($(HAVE_CLIPS),1)
  CFLAGS_CLIPS  = -DHAVE_CLIPS $(shell $(PKGCONFIG) --cflags 'clipsmm-1.0')
  LDFLAGS_CLIPS = $(shell $(PKGCONFIG) --libs 'clipsmm-1.0')
  # The following is required since the GCC 5 libstdc++ deprecates
  # std::auto_ptr according to C++11. This causes warnings since this
  # is used in glibmm, a dependency of clipsmm.
  ifeq ($(call gcc_atleast_version,5,0),1)
    CFLAGS_CLIPS += -Wno-deprecated-declarations
  endif
endif

endif # __buildsys_clips_mk_

