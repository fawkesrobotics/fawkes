#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

### Debugging related options
SILENT = @

### Directories
#BASEDIR ?= $(HOME)/robocup/fawkes
SRCDIR ?= .
OBJDIR = .objs
DEPDIR = $(SRCDIR)/.deps
BINDIR = $(BASEDIR)/bin
LIBDIR = $(BASEDIR)/lib
PLUGINDIR = $(BASEDIR)/plugins
VPATH = $(SRCDIR)
df = $(DEPDIR)/$(*F)

### Programs used, do not mention trivial stuff like ln, rm, ls as per Makefile manual
CC = gcc
MOC = $(QTDIR)/bin/moc

### GCC version information
GCC_VERSION=$(shell LANG=C $CC -v 2>&1 | grep "gcc version" | awk '{ print $3 }')
GCC_VERSION_MAJOR=$(shell LANG=C $(CC) -v 2>&1 | grep "gcc version" | awk '{ print $$3 }' | awk -F. '{ print $$1 }')

### Eclipse
ECLIPSE_VERSION = 5.7
ECLIPSE_ARCH    = $(shell uname -i)_$(shell uname -s | tr "[A-Z]" "[a-z]")
ECLIPSE_PATH    = /usr/local/eclipse_$(ECLIPSE_VERSION)
ECLIPSE_LIBDIR  = $(ECLIPSE_PATH)/lib/i386_linux
ECLIPSE_INCDIR  = $(ECLIPSE_PATH)/include/i386_linux

### CFLAGS, preprocessor, compiler and linker options
LDFLAGS_LIBDIRS = -Wl,-R$(HOME)/fawkes/lib,-R$(HOME)/robocup/fawkes/lib,-R$(HOME)/work/fawkes/lib,-R$(ECLIPSE_LIBDIR)
DEFAULT_INCLUDES = -I$(BASEDIR)/src
CFLAGS_BASE = -Wall -Werror $(DEFAULT_INCLUDES)
LDFLAGS_BASE = -L$(LIBDIR)
LDFLAGS_SHARED = -shared

### colors, to be used as command, not via echo
BLACK		= $(SILENT) tput setaf 0
BG_BLACK	= $(SILENT) tput setab 0
DARKGREY	= $(SILENT) tput bold ; tput setaf 0
LIGHTGREY	= $(SILENT) tput setaf 7
BG_LIGHTGREY	= $(SILENT) tput setab 7
WHITE		= $(SILENT) tput bold ; tput setaf 7
RED		= $(SILENT) tput setaf 1
BG_RED		= $(SILENT) tput setab 1
BRIGHTRED	= $(SILENT) tput bold ; tput setaf 1
GREEN		= $(SILENT) tput setaf 2
BG_GREEN	= $(SILENT) tput setab 2
BRIGHTGREEN	= $(SILENT) tput bold ; tput setaf 2
BROWN		= $(SILENT) tput setaf 3
BG_BROWN	= $(SILENT) tput setab 3
YELLOW		= $(SILENT) tput bold ; tput setaf 3
BLUE		= $(SILENT) tput setaf 4
BG_BLUE		= $(SILENT) tput setab 4
BRIGHTBLUE	= $(SILENT) tput bold ; tput setaf 4
PURPLE		= $(SILENT) tput setaf 5
BG_PURPLE	= $(SILENT) tput setab 5
PINK		= $(SILENT) tput bold ; tput setaf 5
CYAN		= $(SILENT) tput setaf 6
BG_CYAN		= $(SILENT) tput setab 6
BRIGHTCYAN	= $(SILENT) tput bold ; tput setaf 6
NORMAL		= $(SILENT) tput sgr0

