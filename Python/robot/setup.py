#!/usr/bin/env python

from distutils.core import setup,Extension
import distutils.util
import os
import glob

on_cygwin = distutils.util.get_platform().startswith('cygwin')

commonfiles = ['pyerr.cpp']

rssourcefiles = commonfiles + ['robotsim.cpp','robotik.cpp','robotsim_wrap.cxx']
mpsourcefiles = commonfiles + ['motionplanning.cpp','motionplanning_wrap.cxx']
cosourcefiles = commonfiles + ['collide.cpp','collide_wrap.cxx']
rfsourcefiles = commonfiles + ['rootfind.cpp','pyvectorfield.cpp','rootfind_wrap.cxx']

robotSimDir = '../..'
robotSimLibDir = robotSimDir+'/lib'
krisLibraryLibDir = 'lib'
swigOpts = ['-c++']
includeDirs = [robotSimDir,'Library/KrisLibrary','Library','/usr/include','.']

tinyxmlLibDir = 'Library/tinyxml'

assimpDir='Library/assimp--3.0.1270-sdk'
assimpLibDir=assimpDir+'/lib'


#needed for KrisLibrary to link
kllibs = ['KrisLibrary','tinyxml','glpk','glui']
if on_cygwin:
    kllibs[-1] = 'opengl32'
    kllibs.append('glut32')

#needed for RobotSim to link
libs = ['RobotSim']+kllibs+['ode']

setup(name='RobotSim',
      version='1.0',
      description='RobotSim extension module',
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='https://github.com/krishauser/KrisLibrary',
      ext_modules=[Extension('_robotsim',
                             [os.path.join('src',f) for f in rssourcefiles],
                             include_dirs=includeDirs,
                             define_macros=[('TIXML_USE_STL',None),('dDOUBLE',None)],
                             library_dirs=[robotSimLibDir,'Library/KrisLibrary/'+krisLibraryLibDir,tinyxmlLibDir,assimpLibDir,'/usr/local/lib','/usr/lib'],
                             libraries=libs,
                             language='c++'),
                   Extension('_motionplanning',
                             [os.path.join('src',f) for f in mpsourcefiles],
                             include_dirs=includeDirs,
                             library_dirs=[robotSimLibDir,'Library/KrisLibrary/'+krisLibraryLibDir,tinyxmlLibDir,assimpLibDir,'/usr/local/lib','/usr/lib'],
                             libraries=kllibs,
                             language='c++'),
                   Extension('_collide',
                             [os.path.join('src',f) for f in cosourcefiles],
                             include_dirs=includeDirs,
                             library_dirs=['Library/KrisLibrary/'+krisLibraryLibDir,tinyxmlLibDir,assimpLibDir,'/usr/local/lib','/usr/lib'],
                             libraries=kllibs,
                             language='c++'),
                   Extension('_rootfind',
                             [os.path.join('src',f) for f in rfsourcefiles],
                             include_dirs=includeDirs,
                             library_dirs=['Library/KrisLibrary/'+krisLibraryLibDir,tinyxmlLibDir,assimpLibDir,'/usr/local/lib','/usr/lib'],
                             libraries=kllibs,
                             language='c++')],
      py_modules=['robotsim.py','motionplanning.py','collide.py','rootfind.py']
     )

