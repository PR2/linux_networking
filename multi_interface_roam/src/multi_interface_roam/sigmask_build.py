#! /usr/bin/env python

from distutils.core import setup, Extension

setup(name = "sigmask",
        version = "1.0",
        ext_modules = [Extension("sigmask", ["sigmask.c"])])
