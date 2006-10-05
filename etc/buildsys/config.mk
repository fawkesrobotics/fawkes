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
df = $(DEPDIR)/$(subst ._,,$(subst /,_,$(subst ..,__,$(subst ./,,$(*D))))_)$(*F)

INDENT_STRING = -  

### Programs used, do not mention trivial stuff like ln, rm, ls as per Makefile manual
CC = gcc
MOC = $(QTDIR)/bin/moc
DOXYGEN = doxygen

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
CFLAGS_BASE = -Wall -Werror -pthread $(DEFAULT_INCLUDES)
LDFLAGS_BASE = -L$(LIBDIR)
LDFLAGS_SHARED = -shared

### colors, to be used as command, not via echo
BLACK		= tput setaf 0
BG_BLACK	= tput setab 0
DARKGREY	= tput bold ; tput setaf 0
LIGHTGREY	= tput setaf 7
BG_LIGHTGREY	= tput setab 7
WHITE		= tput bold ; tput setaf 7
RED		= tput setaf 1
BG_RED		= tput setab 1
BRIGHTRED	= tput bold ; tput setaf 1
GREEN		= tput setaf 2
BG_GREEN	= tput setab 2
BRIGHTGREEN	= tput bold ; tput setaf 2
BROWN		= tput setaf 3
BG_BROWN	= tput setab 3
YELLOW		= tput bold ; tput setaf 3
BLUE		= tput setaf 4
BG_BLUE		= tput setab 4
BRIGHTBLUE	= tput bold ; tput setaf 4
PURPLE		= tput setaf 5
BG_PURPLE	= tput setab 5
PINK		= tput bold ; tput setaf 5
CYAN		= tput setaf 6
BG_CYAN		= tput setab 6
BRIGHTCYAN	= tput bold ; tput setaf 6
NORMAL		= tput sgr0

