#*****************************************************************************
#              Makefile Build System for Fawkes: ECLiPSe Config
#                            -------------------
#   Created on Wed Jul 15 15:40:56 2009
#   Copyright (C) 2009 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(abspath $(BUILDSYSDIR)/ext/gmsl)
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/ros.mk

ECLIPSE_BINARY = eclipse-clp
ECLIPSE_MIN_VERSION_MAJOR=6
ECLIPSE_MIN_VERSION_MINOR=0

HAVE_ECLIPSE = $(if $(shell which $(ECLIPSE_BINARY) >/dev/null 2>&1; echo $${?/1/}),1,0)

ifeq ($(HAVE_ECLIPSE),1)
  HOSTARCH=$(shell $(ECLIPSE_BINARY) -e "get_flag(hostarch,A),printf(\"%p\", [A])")
  ifeq ($(findstring $(patsubst i%86,i386,$(ARCH)),$(HOSTARCH)),)
    HAVE_ECLIPSE=0
    ECLIPSE_FAIL_REASON="system architecture not supported"
  endif
  ECLIPSE_VERSION=$(shell $(ECLIPSE_BINARY) -e "get_flag(version,A),printf(\"%p\", [A])")
  ECLIPSE_VERSION_SPLITTED=$(call split,.,$(ECLIPSE_VERSION))
  ECLIPSE_VERSION_MAJOR=$(word 1,$(ECLIPSE_VERSION_SPLITTED))
  ECLIPSE_VERSION_MINOR=$(word 2,$(ECLIPSE_VERSION_SPLITTED))
  ifneq ($(call gte,$(ECLIPSE_VERSION_MAJOR),$(ECLIPSE_MIN_VERSION_MAJOR)),$(true))
    HAVE_ECLIPSE=0
    ECLIPSE_FAIL_REASON="Insufficient ECLiPSe version, installed $(ECLIPSE_VERSION), required $(ECLIPSE_MIN_VERSION_MAJOR).$(ECLIPSE_MIN_VERSION_MINOR)"
  else
    ifeq ($(ECLIPSE_VERSION_MAJOR),$(ECLIPSE_MIN_VERSION_MAJOR))
      ifneq ($(call gte,$(ECLIPSE_VERSION_MINOR),$(ECLIPSE_MIN_VERSION_MINOR)),$(true))
        HAVE_ECLIPSE=0
        ECLIPSE_FAIL_REASON="Insufficient ECLiPSe version, installed $(ECLIPSE_VERSION), required $(ECLIPSE_MIN_VERSION_MAJOR).$(ECLIPSE_MIN_VERSION_MINOR)"
      endif
    endif
  endif
else
  ECLIPSE_FAIL_REASON="ECLiPSe not installed"
endif

ifeq ($(call boost-have-lib,thread system filesystem regex),0)
  HAVE_ECLIPSE=0
  ECLIPSE_FAIL_REASON="boost lib filesystem, thread, system or regex not installed"
endif

ifeq ($(HAVE_ECLIPSE),1)
  HAVE_ECL_PACKAGE = $(shell pkg-config --exists eclipse-clp && echo 1 || echo 0)
  ifeq ($(HAVE_ECL_PACKAGE), 1)
    ECLIPSE_LIB = $(shell pkg-config --libs eclipse-clp)
    ECLIPSE_INC = $(shell pkg-config --cflags eclipse-clp)
    ECLIPSE_CFLAGS = $(ECLIPSE_INC) -DECLIPSE_CODE_DIR=\"$(abspath $(BASEDIR)/src/plugins/eclipse-clp)\"
    ECLIPSE_LDFLAGS = $(ECLIPSE_LIB)
  else
    ECLIPSE_LIBDIR = $(shell $(ECLIPSE_BINARY) -e "get_flag(installation_directory,D),printf(\"%p\", [D])")/lib/$(HOSTARCH)
    ECLIPSE_INCDIR = $(shell $(ECLIPSE_BINARY) -e "get_flag(installation_directory,D),printf(\"%p\", [D])")/include/$(HOSTARCH)

    ifeq ($(wildcard $(ECLIPSE_INCDIR)),)
      # Includes are not in the installation directory,
      # might be in system path, check there.
      ifneq ($(wildcard $(SYSROOT)/usr/include/eclipse-clp),)
        ECLIPSE_INCDIR = $(SYSROOT)/usr/include/eclipse-clp
      else
        HAVE_ECLIPSE=
        ECLIPSE_FAIL_REASON="Headers not found"
      endif
    endif

    ECLIPSE_CFLAGS = -I$(ECLIPSE_INCDIR) -DECLIPSE_CODE_DIR=\"$(abspath $(BASEDIR)/src/plugins/eclipse-clp)\"
    ECLIPSE_LDFLAGS = -L$(ECLIPSE_LIBDIR) -Wl,-R$(ECLIPSE_LIBDIR)
  endif
  ECLIPSE_LDFLAGS += $(call boost-libs-ldflags,thread system filesystem regex) \
			-DBASEDIR=\"$(BASEDIR)\"
endif

# the following is needed for quaternion helper class
USE_ROS_BULLET=0
ifeq ($(HAVE_ROS),1)
  ifeq ($(call ros-have-pkg,bullet),1)
    USE_ROS_BULLET=1
  endif
endif

ifeq ($(USE_ROS_BULLET),1)
  HAVE_BULLET=1
  CFLAGS_BULLET  = $(call ros-pkg-cflags,bullet) -Wno-deprecated-declarations
  LDFLAGS_BULLET = $(call ros-pkg-lflags,bullet)
else
  ifneq ($(PKGCONFIG),)
    HAVE_BULLET = $(if $(shell $(PKGCONFIG) --exists 'bullet'; echo $${?/1/}),1,0)
    ifeq ($(HAVE_BULLET),1)
      CFLAGS_BULLET  = $(shell $(PKGCONFIG) --cflags 'bullet')
      # we're only interested in the math part
      LDFLAGS_BULLET = -lLinearMath
    endif
  endif
endif

ifeq ($(HAVE_BULLET),1)
  HAVE_TF = 1
  ECLIPSE_CFLAGS += -DHAVE_TF $(CFLAGS_BULLET)
  ECLIPSE_LDFLAGS += $(LDFLAGS_BULLET) -lm
  ifeq ($(HAVE_CPP11),1)
    CFLAGS_TF += $(CFLAGS_CPP11)
  endif
endif


