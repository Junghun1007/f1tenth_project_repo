from glob import glob
from setuptools import setup
setup(name='longitudinal_autotune', version='0.1.0', packages=['longitudinal_autotune'],
      description='Isolated two-stop-line longitudinal experiments and bounded tuning',
      maintainer='ohslo', maintainer_email='ohslo@example.com', license='Apache-2.0',
      data_files=[('share/ament_index/resource_index/packages', ['resource/longitudinal_autotune']),
                  ('share/longitudinal_autotune', ['package.xml', 'README.md']),
                  ('share/longitudinal_autotune/launch', glob('launch/*.launch.py')),
                  ('share/longitudinal_autotune/config', glob('config/*.yaml'))],
      install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
      entry_points={'console_scripts': ['autotune_session = longitudinal_autotune.node:main',
                                        'autotune_analyze = longitudinal_autotune.analysis:main']})
