#!/usr/bin/env python

from distutils.core import setup,Extension
import os
import glob

#i couldn't get swig to compile c++ files...
os.system('swig -python -c++ collide.i')

sourcefiles =  glob.glob('*.cpp')+glob.glob(os.path.join('PQP','*.cpp'))+glob.glob('*.cxx')

swigOpts = ['-c++']


setup(name='CollideExtension',
      version='1.0',
      description='C++ collision testing  extension',
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='http://code.google.com/p/pytamp/',
      ext_modules=[Extension('_collide',
                             sourcefiles,
                             language='c++')],
      py_modules=['collide']
     )

