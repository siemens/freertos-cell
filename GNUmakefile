.PHONY: all clean distclean

all:
	git submodule update --init
	$(MAKE) -f Makefile -j `nproc`

clean distclean:
	git submodule foreach git clean -d -x -f
	$(MAKE) -f Makefile $@
