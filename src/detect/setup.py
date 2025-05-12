from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 自动生成 distutils 所需的配置
d = generate_distutils_setup(
    # 定义 Python 包的名称
    packages=['detect','new_detect_function'],
    # 指定 Python 包的根目录
    package_dir={'': 'src'},

)

# 使用生成的配置进行 setup
setup(**d)

