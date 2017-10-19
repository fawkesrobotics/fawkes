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

CLIPS_ERROR=

ifneq ($(PKGCONFIG),)
  ifeq ($(HAVE_CPP14),1)
    HAVE_CLIPS = $(if $(shell $(PKGCONFIG) --exists 'clipsmm-1.0'; echo $${?/1/}),1,0)
    ifneq ($(HAVE_CLIPS),1)
      CLIPS_ERROR = CLIPS not found
    endif
  else
    CLIPS_ERROR = C++14 required
  endif
else
  CLIPS_ERROR = pkg-config not available
endif

ifeq ($(HAVE_CLIPS),1)
  # Filter out "-std=c++0x" for clipsmm <=0.3.4 (it unnecessarily downgrades the std)
  CFLAGS_CLIPS  = -DHAVE_CLIPS $(CFLAGS_CPP14) \
                  $(filter-out -std=c++0x,$(shell $(PKGCONFIG) --cflags 'clipsmm-1.0'))
  LDFLAGS_CLIPS = $(shell $(PKGCONFIG) --libs 'clipsmm-1.0')

  # The following is required since C++11 deprecates some features triggering
  # warnings in older glibmm version (before 2.46):
  # -Wno-deprecated-declarations is needed due to the use ofstd::auto_ptr
  # -Wno-deprecated is needed with GCC7 because glibmm still uses throw()
  # specifications, which are deprecated and cause a GCC warning.
  ifneq ($(if $(shell $(PKGCONFIG) --atleast-version 2.46 'glibmm-2.4'; echo $${?/1/}),1,0),1)
    $(warning HERE)
    CFLAGS_CLIPS += -Wno-deprecated-declarations -Wno-deprecated
  endif
endif

endif # __buildsys_clips_mk_

