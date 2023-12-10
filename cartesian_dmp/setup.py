from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    scripts=['src/cartesian_dmp/cartesian_dmp.py', 'src/cartesian_dmp/cartesian_dmp.py'],
    packages=['cartesian_dmp'],
    package_dir={'': 'src'}
)
setup(**d)