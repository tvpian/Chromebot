#! /usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["chromebot_controller"],
    package_dir={"": "scripts"},    
)

# e = generate_distutils_setup(
#     packages=["bumberbot_examples"],
#     package_dir={"": "srv"},
# )


setup(**d)
# setup(**e)