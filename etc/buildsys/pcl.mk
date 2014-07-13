#*****************************************************************************
#                 Makefile Build System for Fawkes: PCL bits
#                            -------------------
#   Created on Mon Nov 07 01:06:16 2011
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
$(error config.mk must be included before pcl.mk)
endif

ifndef __buildsys_pcl_mk_
__buildsys_pcl_mk_ := 1


# It might be a ROS PCL installation, we need to pull in std_msgs
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/eigen3.mk

PCL_LIB_DIRS=$(shell ld --verbose 2>&1 | grep SEARCH | sed 's/SEARCH_DIR("\([^"]*\)");/\1/g')

ifneq ($(PKGCONFIG),)
  HAVE_PCL = $(if $(shell $(PKGCONFIG) --exists 'pcl_common'; echo $${?/1/}),1,0)
endif

ifneq ($(HAVE_EIGEN3),1)
  HAVE_PCL = 0
endif

ifneq ($(wildcard $(SYSROOT)/usr/include/vtk/vtkVersion.h),)
  HAVE_VTK = 1
  CFLAGS_VTK = -I/usr/include/vtk
endif

ifneq ($(HAVE_PCL),1)
  # Give it another shot, name might contain version
  _PCL_ALTERNATE_NAME=$(shell $(PKGCONFIG) pkg-config --list-all | grep pcl_common | awk '{ print $$1 }')
  ifneq ($(_PCL_ALTERNATE_NAME),)
    PCL_VERSION_SUFFIX=$(patsubst pcl_common%,%,$(lastword $(_PCL_ALTERNATE_NAME)))
    HAVE_PCL=1
  endif
endif

ifeq ($(HAVE_PCL),1)
  # if we have ROS, by default use std_msgs and sensor_msgs from there, rather
  # than the types that come with PCL, because it'll crash miserably when pumping
  # data from Fawkes to ROS, it might cause problems the other way around
  # ifeq ($(HAVE_ROS),1)
  #   CFLAGS_PCL  = $(call ros-pkg-cflags,sensor_msgs) \
  # 		  $(call ros-pkg-cflags,std_msgs) \
  # 		  -DHAVE_ROS_STD_MSGS
  #   LDFLAGS_PCL = $(call ros-pkg-lflags,sensor_msgs) \
  # 		  $(call ros-pkg-lflags,std_msgs) \
  # 		  -DHAVE_ROS_SENSOR_MSGS
  # endif

  PCL_VERSION  = $(shell $(PKGCONFIG) --modversion 'pcl_common$(PCL_VERSION_SUFFIX)')

  CFLAGS_PCL  += -DHAVE_PCL $(CFLAGS_EIGEN3) \
		 $(shell $(PKGCONFIG) --cflags 'pcl_common$(PCL_VERSION_SUFFIX)') \
		 -Wno-unknown-pragmas -Wno-deprecated-declarations
  LDFLAGS_PCL += $(LDFLAGS_EIGEN3) \
		 $(if $(HAVE_ROS),-L$(shell $(PKGCONFIG) --variable libdir 'pcl_common$(PCL_VERSION_SUFFIX)')) \
		 $(shell $(PKGCONFIG) --libs 'pcl_common$(PCL_VERSION_SUFFIX)')

  ifeq ($(CC),clang)
    CFLAGS_PCL += -Wno-overloaded-virtual
  endif

  ROS_PCL=$(filter /opt/ros%,$(shell $(PKGCONFIG) --variable=prefix 'pcl_common$(PCL_VERSION_SUFFIX)'))
  ifneq ($(ROS_PCL),)
    ifneq ($(HAVE_ROS),1)
      HAVE_PCL=
      PCL_ERROR = ROS PCL installation, but ROS not found
    else
      ifneq ($(call ros-have-pkg,std_msgs),1)
        HAVE_PCL=
        PCL_ERROR = ROS PCL installation, but std_msgs not found
      else
        CFLAGS_PCL  += -DHAVE_ROS_PCL $(CFLAGS_ROS)  $(call ros-pkg-cflags,std_msgs)
        LDFLAGS_PCL += $(LDFLAGS_ROS) $(call ros-pkg-lflags,std_msgs)
      endif
    endif
  endif

  PCL_COMMON_LIB = $(firstword $(wildcard $(addsuffix /libpcl_common.$(SOEXT),$(PCL_LIB_DIRS))))
  ifneq ($(PCL_COMMON_LIB),)
    ifneq ($(OPENMP_LIBRARY),)
      PCL_USES_OPENMP = $(if $(shell ldd $(PCL_COMMON_LIB) | grep lib$(OPENMP_LIBRARY).$(SOEXT)),1)
    endif
  endif

  pcl-version-atleast = $(if $(shell $(PKGCONFIG) --atleast-version=$1 'pcl_common$(PCL_VERSION_SUFFIX)'; echo $${?/1/}),1,)
  pcl-version-exact = $(if $(shell $(PKGCONFIG) --exact-version=$1 'pcl_common$(PCL_VERSION_SUFFIX)'; echo $${?/1/}),1,)
  pcl-version-max = $(if $(shell $(PKGCONFIG) --max-version=$1 'pcl_common$(PCL_VERSION_SUFFIX)'; echo $${?/1/}),1,)
  pcl-have-lib    = $(if $(shell $(PKGCONFIG) --exists 'pcl_$1$(PCL_VERSION_SUFFIX)'; echo $${?/1/}),1,0)
  pcl-lib-cflags  = $(shell $(PKGCONFIG) --cflags 'pcl_$1$(PCL_VERSION_SUFFIX)')
  pcl-lib-ldflags = $(shell $(PKGCONFIG) --libs 'pcl_$1$(PCL_VERSION_SUFFIX)')

  pcl-have-libs    = $(if $(strip $(subst 1,,$(foreach c,$1,$(call pcl-have-lib,$c)))),0,1)
  pcl-libs-cflags  = $(shell $(PKGCONFIG) --cflags $(foreach c,$1,'pcl_$c$(PCL_VERSION_SUFFIX)' ))
  pcl-libs-ldflags = $(shell $(PKGCONFIG) --libs $(foreach c,$1,'pcl_$c$(PCL_VERSION_SUFFIX)' ))

  pcl-missing-libs = $(strip $(foreach c,$1,$(if $(subst 1,,$(call pcl-have-lib,$c)),$c)))

else
  pcl-have-lib = 0
endif

endif # __buildsys_pcl_mk_

