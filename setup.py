from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os


package_name = "utils_ros"
package_dir = "python"

packages = []
for dir in os.listdir(package_dir):
    if "__init__.py" in os.listdir(os.path.join(package_dir, dir)):
        packages.append(dir)

d = generate_distutils_setup(
    packages=packages,
    package_dir={'': package_dir},
    scripts=[]  #scripts are installed to GLOBAL_BIN_DIR! Do NOT use.
)

setup(**d)
