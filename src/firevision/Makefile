#*****************************************************************************
#               Makefile Build System for Fawkes: FireVision
#                            -------------------
#   Created on Sun Jan 14 18:24:42 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../..
include $(BASEDIR)/src/firevision/fvconf.mk

SUBDIRS = fvutils cams models filters classifiers stereo fvwidgets apps tools

ifeq ($(HAVE_SIFT),1)
SUBDIRS += extlib/sift
CLASSIFIERS_DEPS += extlib/sift
endif
ifeq ($(HAVE_SURF),1)
SUBDIRS += extlib/surf
CLASSIFIERS_DEPS += extlib/surf
endif

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/rules.mk
include $(BASEDIR)/etc/buildsys/download.mk

cams models filters classifiers stereo fvwidgets: fvutils
filters: models
apps: fvutils models cams filters classifiers stereo
tools: apps fvutils stereo
models: classifiers
ifneq ($(CLASSIFIERS_DEPS),)
classifiers: $(CLASSIFIERS_DEPS)
endif

# http://web.engr.oregonstate.edu/~hess/downloads/sift/sift-latest.tar.gz
SIFT_FILE = sift-latest.tar.gz
URLS_sift = http://robocup.rwth-aachen.de/files/sourcecode/$(SIFT_FILE) \
	    http://robocup.rwth-aachen.de/files/sourcecode/sift-fawkes.patch

# http://www.vision.ee.ethz.ch/~surf/SURF-V1.0.9.tar.gz
SURF_VERSION = 1.0.9
SURF_FILE = SURF-V$(SURF_VERSION).tar.gz
SURF_UNPACKDIR = SURF-V$(SURF_VERSION)
URLS_surf = http://robocup.rwth-aachen.de/files/sourcecode/$(SURF_FILE) \
	    http://robocup.rwth-aachen.de/files/sourcecode/surf-fawkes.patch

.PHONY: create-extlibdir download-sift download-surf
create-extlibdir:
	$(SILENT)mkdir -p $(FVEXTLIBDIR)

download-sift: create-extlibdir
	$(SILENT)cd $(FVEXTLIBDIR); \
	$(download-files)

download-surf: create-extlibdir
	$(SILENT)cd $(FVEXTLIBDIR); \
	$(download-files)


.PHONY: get-sift get-surf
get-sift: download-sift
	$(SILENT)cd $(FVEXTLIBDIR); \
	rm -rf sift; \
	echo -e "$(INDENT_PRINT)--- Unpacking $(SIFT_FILE)"; \
	tar xfz $(SIFT_FILE); \
	echo -e "$(INDENT_PRINT)--- Patching SIFT lib for Fawkes compatibility"; \
	cd sift; \
	patch -p1 < ../sift-fawkes.patch; \
	mkdir include/sift; \
	mv include/*.h include/sift;
	$(SILENT)$(MAKE) -C extlib/sift INDENT="$(INDENT)$(INDENT_STRING)"

ifeq ($(ARCH),x86_64)
get-surf:
	$(SILENT) echo -e "$(INDENT_PRINT)--- $(TRED)SURF is not available for x86_64$(TNORMAL)"
else
get-surf: download-surf
	$(SILENT)cd $(FVEXTLIBDIR); \
	rm -rf surf SURF; \
	echo -e "$(INDENT_PRINT)--- Unpacking $(SURF_FILE)"; \
	tar xfz $(SURF_FILE); \
	echo -e "$(INDENT_PRINT)--- Patching SURF lib for Fawkes compatibility"; \
	mv $(SURF_UNPACKDIR) surf; \
	cd surf; \
	patch -p1 < ../surf-fawkes.patch
	$(SILENT)$(MAKE) -C extlib/surf
endif

