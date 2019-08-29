ROOTDIR = ../..
TEMPLATEDIR = $(ROOTDIR)/scripts/header_gen/svd_templates
SCRIPTDIR = $(ROOTDIR)/scripts/header_gen
SOURCEDIR = $(ROOTDIR)/mcu/apollo/regs/source
SOURCEDIR2 = $(ROOTDIR)/mcu/apollo2/regs/source
SOURCEDIR3 = $(ROOTDIR)/mcu/apollo3/regs/source

SVDFILES = apollo1.svd apollo2.svd apollo3.svd

.PHONY: all clean realclean
all: $(SVDFILES)

realclean: clean

clean:
	rm -f $(SVDFILES)

apollo1.svd: $(SOURCEDIR)/**
	python3 $(SCRIPTDIR)/svd_gen.py -o $@ -t $(TEMPLATEDIR) $(SOURCEDIR) -n apollo1

apollo2.svd: $(SOURCEDIR2)/**
	python3 $(SCRIPTDIR)/svd_gen.py -o $@ -t $(TEMPLATEDIR) $(SOURCEDIR2) -n apollo2 --skip_fieldname_check

apollo3.svd: $(SOURCEDIR3)/**
	python3 $(SCRIPTDIR)/svd_gen.py -o $@ -t $(TEMPLATEDIR) $(SOURCEDIR3) -n apollo3
