.PHONY: docs lib clean

default: lib

docs: 
	doxygen doxygen.conf
	./doxy2swig.py docs/xml/index.xml docs/docs.i
	pydoc -w robotsim

lib:
	ln -sf ../../Library
	python setup.py build_ext --inplace

clean:
	rm -f *.pyc
	rm -f *.so
	rm -f *.dll
	rm -f doxygen_warnings.log
	rm -rf docs



