from glob import glob
from setuptools import setup

setup(name='tunnel_drive', version='0.1.0', packages=['tunnel_drive'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/tunnel_drive']),
                  ('share/tunnel_drive', ['package.xml', 'README.md']),
                  ('share/tunnel_drive/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['tunnel_lane_selector = tunnel_drive.node:main']})
