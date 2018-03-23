#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Mon 24 Apr 2017 17:39:49 CEST
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
include $(BUILDCONFDIR)/tf/tf.mk
include $(FAWKES_BASEDIR)/src/plugins/mongodb/mongodb.mk
include $(BUILDSYSDIR)/boost.mk

REQ_BOOST_LIBS = bind function
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_MONGODB)$(HAVE_TF)$(HAVE_BOOST_LIBS),111)
  HAVE_ROBOT_MEMORY = 1
  ifeq ($(DISTRO),ubuntu)
    OLDER_THAN_XENIAL=$(shell [[ "$(DISTRO_VERSION)" < "16.04" ]] && echo 1)
    ifeq ($(OLDER_THAN_XENIAL),1)
      HAVE_ROBOT_MEMORY = 0
    endif
  endif
  CFLAGS_ROBOT_MEMORY = $(CFLAGS_TF) $(CFLAGS_MONGODB) \
                        $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS_ROBOT_MEMORY = $(LDFLAGS_TF) $(LDFLAGS_MONGODB) \
                         $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
endif
