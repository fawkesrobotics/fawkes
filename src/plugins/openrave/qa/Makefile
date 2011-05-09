
BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

LIBS_qa_or_env = fawkesutils fawkesopenrave stdc++
OBJS_qa_or_env = qa_env.o

LIBS_qa_or_manip = fawkesutils fawkesopenrave stdc++
OBJS_qa_or_manip = qa_manip.o

LIBS_qa_or_robot = fawkescore fawkesutils fawkesopenrave stdc++
OBJS_qa_or_robot = qa_robot.o

LIBS_qa_or_con = fawkesutils fawkesopenrave stdc++
OBJS_qa_or_con = qa_connector.o

OBJS_all = $(OBJS_qa_or_con)) \
		$(OBJS_qa_or_env) \
		$(OBJS_qa_or_manip) \
		$(OBJS_qa_or_robot)


BINS_all = $(BINDIR)/qa_or_con \
		$(BINDIR)/qa_or_env \
		$(BINDIR)/qa_or_manip \
		$(BINDIR)/qa_or_robot


include $(BUILDSYSDIR)/base.mk

