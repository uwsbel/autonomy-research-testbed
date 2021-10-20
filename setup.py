"""A setuptools based setup module."""

# Always prefer setuptools over distutils
from setuptools import setup, find_packages
import pathlib

here = pathlib.Path(__file__).parent.resolve()

# Get the long description from the README file
long_description = (here / 'README.md').read_text(encoding='utf-8')

# Arguments marked as "Required" below must be included for upload to PyPI.
# Fields marked as "Optional" may be commented out.

setup(
    name='miniav',  # Required
    version='0.0.0',  # Required
    description='MiniAV Project',  # Optional
    long_description=long_description,  # Optional
    long_description_content_type='text/markdown',  # Optional (see note above)
    author='UW Simulation Based Engineering Lab',  # Optional
    packages=find_packages(),  # Required
    python_requires='>=3.6, <4',
    install_requires=[''],  # Optional
    entry_points={  # Optional
        'console_scripts': [
            'miniav=miniav.miniav_base:main',
        ],
    },
)
