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


ifneq ($(PKGCONFIG),)
  HAVE_CLIPS = $(if $(shell $(PKGCONFIG) --exists 'clipsmm-1.0'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_CLIPS),1)
  CFLAGS_CLIPS  = -DHAVE_CLIPS $(shell $(PKGCONFIG) --cflags 'clipsmm-1.0')
  LDFLAGS_CLIPS = $(shell $(PKGCONFIG) --libs 'clipsmm-1.0')
endif

endif # __buildsys_clips_mk_

