#!/usr/bin/env python

from distutils.core import setup,Extension
import os
import glob

#i couldn't get swig to compile c++ files...
os.system('swig -python -c++ rootfind.i')

sourcefiles =  glob.glob('*.cpp')+glob.glob('*.cxx')

swigOpts = ['-c++']


setup(name='RootFind',
      version='1.0',
      description='Newton-Raphson Root Finding of Vector Equality',
      author='Mark Wilson',
      author_email='mw54@indiana.edu',
      url='http://code.google.com/p/pytamp/',
      package_data = {'rootfind' : ['rootfind.py'] },
      ext_modules=[Extension('_rootfind',
                             sourcefiles,
                             define_macros=[('HAVE_IEEE_COMPARISONS','1')],
                             language='c++')],
      py_modules=['rootfind']
     )
