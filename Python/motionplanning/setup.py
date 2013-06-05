#!/usr/bin/env python

from distutils.core import setup,Extension
import os
import glob

#i couldn't get swig to compile c++ files...
os.system('swig -python -c++ motionplanning.i')

sourcefiles =  glob.glob('*.cpp')+glob.glob('*.cxx')

swigOpts = ['-c++']


setup(name='MotionPlanning',
      version='1.0',
      description='Motion Planning',
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='http://code.google.com/p/pytamp/',
      package_data = {'motionplanning' : ['motionplanning.py'] },
      ext_modules=[Extension('_motionplanning',
                             sourcefiles,
                             define_macros=[('HAVE_IEEE_COMPARISONS','1')],
                             language='c++')],
      py_modules=['motionplanning']
     )
