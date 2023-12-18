from setuptools import find_packages
from setuptools import setup

setup(
    name='pappap',
    version='0.0.0',
    packages=find_packages(
        include=('pappap', 'pappap.*')),
)
