#!/usr/bin/env python

from distutils.core import setup

setup(name='Sensors3140',
      version='0.0.1',
      description='Sensors for FRC',
      author='David Bolme',
      author_email='dbolme@gmail.com',
      #url='https://www.python.org/sigs/distutils-sig/',
      package_dir={'':'python',},
      packages=['sensors3140', 'sensors3140.calibrate'],
     )