SWIG = swig

include ../Makefile.config
SRCS= $(wildcard *.cpp)
LIBNAME= RobotSim
INCDIR=
DEFINES += $(MATH_DEFINES)

STD_INCLUDES= /usr/include/
STD_LIBS=
PY_INCLUDES= /usr/include/python2.6/
PY_LIBDIR= /usr/lib/
#PY_INCLUDES= /l/python2.7/include/python2.7/
#PY_LIBDIR= /l/python2.7/lib/

INCDIR = . $(PY_INCLUDES) $(INCDIRS)

CPPFLAGS = -Wall -fPIC

include ../Makefile.template

.PHONY: clean

default: $(OBJS) robotsim_wrap.o

robotsim_wrap.o: robotsim_wrap.cxx 
	g++  $(FLAGS) robotsim_wrap.cxx -o $(OBJDIR)/robotsim_wrap.o -I$(PY_INCLUDES) -c

robotsim_wrap.cxx: docs robotsim.i robotsim.h robotmodel.h robotik.h robotik.cpp pyerr.h
	$(SWIG) -c++ -python robotsim.i

clean:
	-rm -rf $(OBJDIR)
	-rm robotsim_wrap.cxx

