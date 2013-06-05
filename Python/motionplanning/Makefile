.PHONY: lib clean

default: lib
	@echo Done.

lib:
	python setup.py build_ext --inplace

clean:
	rm -f *.pyc
	rm -f *.so
	rm -f *.dll
	rm -f doxygen_warnings.log
	rm -rf docs


