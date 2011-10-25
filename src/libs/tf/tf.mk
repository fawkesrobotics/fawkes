#*****************************************************************************
#        Makefile Build System for Fawkes: Transforms Library Config
#                            -------------------
#   Created on Tue Oct 18 16:23:47 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************


ifneq ($(PKGCONFIG),)
  HAVE_BULLET = $(if $(shell $(PKGCONFIG) --exists 'bullet'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_BULLET),1)
    CFLAGS_BULLET  = $(shell $(PKGCONFIG) --cflags 'bullet')
    # we're only interested in the math part
    LDFLAGS_BULLET = -lLinearMath
  endif
endif

ifeq ($(HAVE_BULLET),1)
  HAVE_TF = 1
  CFLAGS_TF  = -DHAVE_TF -std=c++0x $(CFLAGS_BULLET) \
	       -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX
  LDFLAGS_TF = $(LDFLAGS_BULLET) -lm
endif
