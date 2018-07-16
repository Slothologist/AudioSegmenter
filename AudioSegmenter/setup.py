from setuptools import find_packages
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='audio_segmenter',
    version='0.1',
    description='Segmentation of audio for a speech pipeline',
    url='---none---',
    author='rfeldhans',
    author_email='rfeldhans@techfak.uni-bielefeld.de',
    license='---none---',
    packages=find_packages())

setup(**setup_args)