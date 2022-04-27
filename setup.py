import os
import setuptools
import sys
from setuptools import setup, find_packages

def read(filename):
    return open(os.path.join(os.path.dirname(__file__), filename)).read()

setup(
    name = 'openBlimp',
    packages = find_packages('src'),
    package_dir = {'': 'src'},
    description = 'OpenBlimp Code and Workshop Demos',
    install_requires = ['numpy', 'pandas', 'pygame', 'opencv-python', 'pyserial', 'scipy']
)
