#*****************************************************************************
#          Makefile Build System for Fawkes: External Libraries
#                            -------------------
#   Created on Thu Jan 13 19:10:53 2011
#   copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDSYSDIR)/download.mk

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

# http://vision.ucla.edu/~vedaldi/code/siftpp/assets/siftpp/versions/siftpp-0.8.0.tar.gz
SIFTPP_VERSION = 0.8.0
SIFTPP_FILE = siftpp-$(SIFTPP_VERSION).tar.gz
SIFTPP_UNPACKDIR = siftpp
URLS_siftpp = http://robocup.rwth-aachen.de/files/sourcecode/$(SIFTPP_FILE) \
            http://robocup.rwth-aachen.de/files/sourcecode/siftpp-fawkes.patch

.PHONY: create-extlibdir download-sift download-surf download-siftpp
create-extlibdir:
	$(SILENT)mkdir -p $(EXTLIBDIR)

download-sift: create-extlibdir
	$(SILENT)cd $(EXTLIBDIR); \
	$(download-files)

download-surf: create-extlibdir
	$(SILENT)cd $(EXTLIBDIR); \
	$(download-files)

download-siftpp: create-extlibdir
	$(SILENT)cd $(EXTLIBDIR); \
	$(download-files)


.PHONY: get-sift get-surf get-siftpp
get-sift: download-sift
	$(SILENT)cd $(EXTLIBDIR); \
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
	$(SILENT)cd $(EXTLIBDIR); \
	rm -rf surf SURF; \
	echo -e "$(INDENT_PRINT)--- Unpacking $(SURF_FILE)"; \
	tar xfz $(SURF_FILE); \
	echo -e "$(INDENT_PRINT)--- Patching SURF lib for Fawkes compatibility"; \
	mv $(SURF_UNPACKDIR) surf; \
	cd surf; \
	patch -p1 < ../surf-fawkes.patch
	$(SILENT)$(MAKE) -C extlib/surf
endif

get-siftpp: download-siftpp
	$(SILENT)cd $(EXTLIBDIR); \
	rm -rf siftpp; \
	echo -e "$(INDENT_PRINT)--- Unpacking $(SIFTPP_FILE)"; \
	tar xfz $(SIFTPP_FILE); \
	echo -e "$(INDENT_PRINT)--- Patching SIFTPP lib for Fawkes compatibility"; \
	cd siftpp; \
	patch -p1 < ../siftpp-fawkes.patch
	$(SILENT)$(MAKE) -C extlib/siftpp
