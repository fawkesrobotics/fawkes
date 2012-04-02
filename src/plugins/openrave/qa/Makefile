
BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

LIBS_qa_or_env = fawkeslogging fawkesopenrave stdc++
OBJS_qa_or_env = qa_env.o

LIBS_qa_or_manip = fawkesopenrave stdc++
OBJS_qa_or_manip = qa_manip.o

LIBS_qa_or_robot = fawkescore fawkeslogging fawkesopenrave stdc++
OBJS_qa_or_robot = qa_robot.o

LIBS_qa_or_modules = fawkescore fawkeslogging fawkesopenrave stdc++
OBJS_qa_or_modules = qa_modules.o

LIBS_qa_or_con = fawkesutils fawkesopenrave stdc++
OBJS_qa_or_con = qa_connector.o

CFLAGS  += $(shell openrave-config --cflags-only-I) -Wno-deprecated -Wno-deprecated-declarations -Wno-unused-but-set-variable
LDFLAGS += $(shell openrave-config --libs-core)

OBJS_all = 	$(OBJS_qa_or_env) \
		$(OBJS_qa_or_manip) \
		$(OBJS_qa_or_robot) \
		$(OBJS_qa_or_modules)


BINS_all = 	$(BINDIR)/qa_or_env \
		$(BINDIR)/qa_or_manip \
		$(BINDIR)/qa_or_robot \
		$(BINDIR)/qa_or_modules


include $(BUILDSYSDIR)/base.mk

