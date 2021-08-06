from setuptools import setup, find_packages

AUTHOR = 'Andrei Gramakov'
EMAIL = 'alexey.gerenkov@espressif.com, andrei.gramakov@espressif.com'
LICENSE = 'MIT'
MAINTAINER = 'Alexey Gerenkov, Andrei Gramakov'
NAME = 'debug_backend'

PACKAGE_DATA = {
    '': ['*.gdb']
}

REQUIREMENTS = [
    'pygdbmi',
    'unittest-xml-reporting'
]

SCRIPTS = [  # All scripts must be listed here
    'scripts/run_tests.py'
]

SHORT_DESCRIPTION = 'This package provides extended capabilities to interacting with ESP chips via GDB and OpenOCD.'
URL = ''
VERSION = '1.0.0'

try:  # Using  README.md as a long description
    with open('README.md') as readme:
        LONG_DESCRIPTION = '/n' + readme.read()
except IOError:
    # maybe running setup.py from some other dir
    LONG_DESCRIPTION = ''

setup(
    author=AUTHOR,
    author_email=EMAIL,
    classifiers=['Programming Language :: Python :: 3'],
    description=SHORT_DESCRIPTION,
    include_package_data=True,
    install_requires=REQUIREMENTS,
    license=LICENSE,
    long_description=LONG_DESCRIPTION,
    maintainer=MAINTAINER,
    name=NAME,
    package_data=PACKAGE_DATA,
    packages=find_packages(),
    platforms='Cross Platform',
    scripts=SCRIPTS,
    url=URL,
    version=VERSION,
)
