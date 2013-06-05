#ifndef ERRORS_H
#define ERRORS_H

#include <stdio.h>
#include <stdarg.h>
#include "assert.h"

inline void RaiseErrorFmt(const char* func, const char* file, int line, const char* fmt, ...)
{
  fprintf(stderr,"Error in %s (%s:%d): ", func,file,line); 
  va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
  fprintf(stderr,"\n");
  abort();
}

inline void RaiseErrorFmt(const char* fmt,...)
{
  fprintf(stderr,"Error (unknown function): ");
  va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
  fprintf(stderr,"\n");
  abort();
}


inline void RaiseError(const char* func, const char* file, int line, const char* text)
{
  fprintf(stderr,"Error in %s (%s:%d): %s\n", func,file,line,text); 
  abort();
}



//the following is bending over backwards to support MS's lack of 
//variable argument macro support

#ifdef HAVE_PRETTY_FUNCTION
#define WHERE_AM_I __PRETTY_FUNCTION__, __FILE__, __LINE__
#else
#define WHERE_AM_I __FUNCTION__, __FILE__, __LINE__
#endif

//Error1 is guaranteed to print line numbers with a single-argument error
#define FatalError1(text) RaiseError(WHERE_AM_I,text)

#if HAVE_VARARGS_MACROS
#define FatalError(fmt,...) RaiseErrorFmt(WHERE_AM_I,fmt,__VA_ARGS__)
#else
//if no variable arguments, can't get any line info 
#define FatalError RaiseErrorFmt
#endif

#define PrintLocation(file)  fprintf(file,"%s (%s:%d): ", WHERE_AM_I)
#define AssertNotReached() RaiseError(WHERE_AM_I,"Code should not be reached")

#endif

