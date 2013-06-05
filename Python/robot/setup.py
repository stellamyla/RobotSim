#!/usr/bin/env python

from distutils.core import setup,Extension
import distutils.util
import os
import glob

on_cygwin = distutils.util.get_platform().startswith('cygwin')

#i couldn't get swig to compile c++ files...
os.system('cp '+os.path.join('src','robotsim.py')+' .')
os.system('swig -python -c++ '+os.path.join('src','robotsim.i'))

subdirs = ['src']
sourcefiles = []
for s in subdirs:
    sourcefiles += glob.glob(os.path.join(s,'*.cpp'))
sourcefiles +=glob.glob(os.path.join('src','*.cxx'))

robotSimDir = '../..'
robotSimLibDir = robotSimDir+'/lib'
krisLibraryLibDir = 'lib'
swigOpts = ['-c++']
includeDirs = [robotSimDir,'Library/KrisLibrary','Library','/usr/include','.']

tinyxmlLibDir = 'Library/tinyxml'

assimpDir='Library/assimp--3.0.1270-sdk'
assimpLibDir=assimpDir+'/lib'


libs = ['RobotSim','KrisLibrary','assimp','tinyxml','ode','gsl','GL', 'glpk', 'glui']
if on_cygwin:
    libs[-1] = 'opengl32'
    libs.append('glut32')

setup(name='RobotSim',
      version='1.0',
      description='RobotSim extension module',
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='http://code.google.com/p/pytamp/',
      ext_modules=[Extension('_robotsim',
                             sourcefiles,
                             include_dirs=includeDirs,
                             define_macros=[('TIXML_USE_STL',None),('dDOUBLE',None)],
                             library_dirs=[robotSimLibDir,'Library/KrisLibrary/'+krisLibraryLibDir,tinyxmlLibDir,assimpLibDir,'/usr/local/lib','/usr/lib'],
                             libraries=libs,
                             language='c++')],
      py_modules=['robotsim.py']
     )

