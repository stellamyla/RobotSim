.PHONY: docs lib


lib: 
	python setup.py build_ext --inplace

clean:
	-rm -rf docs

docs:
	doxygen doxygen.conf

