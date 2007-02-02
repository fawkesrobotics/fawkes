#
# File: Makefile
#
# Written by Tim Niemueller <tim@niemueller.de>
# Created on 2002/08/15
#

DOCCOUNT = $(shell grep -l "begin{document}" *.tex | wc -l)
ifeq ($(DOCCOUNT),1)
DOCNAME = $(subst .tex,,$(shell grep -l "begin{document}" *.tex))
else
$(error Document not found or ambigious)
endif

LATEX=pslatex
PDFLATEX=pdflatex
BIBTEX=bibtex
DVIPS=dvips
DVIPDF=dvipdf
GV=gv
EDITOR ?= vim
XDVI=xdvi
CVS=cvsuni
PDFVIEW=xpdf

SILENT=@
ECHO=$(SILENT)echo

DVIPSFLAGS =
DVIPDFFLAGS =

PSS=$(shell ls *.tex | sed -e "s/\.tex$$/.ps/g")
PDFS=$(shell ls *.tex | sed -e "s/\.tex$$/.pdf/g")

CUBEPAGES=16 38 42 52 61 74 80

all: cache-clean $(DOCNAME).pdf

all-ps: cache-clean
	$(MAKE) $(DOCNAME).ps
	if [ -e $(DOCNAME).bib ]; then \
		$(MAKE) bibtex; rm $(DOCNAME).dvi; $(MAKE) $(DOCNAME).dvi;\
		$(MAKE) $(DOCNAME).ps ;\
	fi

status:
	$(CVS) status | grep File | grep -v Up-to-date

postscripts: $(PSS)
pdfs: $(PDFS)

view: cache-clean all
	$(GV) $(DOCNAME).ps

viewdvi: cache-clean $(DOCNAME).dvi
	$(XDVI) $(DOCNAME).dvi

edit:
	$(EDITOR) $(DOCNAME).tex
	$(MAKE) view

bibtex: $(DOCNAME).bib $(DOCNAME).aux
	$(BIBTEX) $(DOCNAME)

cache-clean:
	$(SILENT)-rm -rf *.log *.ps *.pdf *.dvi 

clean: cache-clean
	$(ECHO) "-- Cleaning up"
	$(SILENT)-rm -rf *.aux *.bbl *.toc *.prv *.blg *.nav

%.dvi: %.tex
	$(ECHO) "--> Creating $@"
	$(LATEX) $<

%.ps: %.dvi
	$(ECHO) "--> Creating $@"
	$(DVIPS) $(DVIPSFLAGS) -o $@ $<

%.pdf: %.dvi
	$(ECHO) "--> Creating $@"
	$(DVIPDF) $(DVIPDFFLAGS) $< $@

view-license: ../gfdl.pdf
	$(PDFVIEW) ../gfdl.pdf

cubepresentation:
	$(SILENT)pdfcube $(DOCNAME).pdf $(CUBEPAGES)

