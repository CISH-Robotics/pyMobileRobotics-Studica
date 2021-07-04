from setuptools import find_packages
from setuptools import setup

setup(name='pyMobileRobotics_Studica',
      version='0.0.1',
      description='pyMobileRobotics Studica Robot Toolkit',
      author='CISH Robotics',
      author_email='crt@cish.xyz',
      url='https://github.com/CISH-Robotics',
      keywords='robot worldskills mobile robotics studica',
      project_urls={
            'Source': 'https://github.com/CISH-Robotics/pyMobileRobotics-Studica',
      },
      packages=find_packages(),
      install_requires=[
            'pyMobileRobotics>=0.0.5'
            ],
      python_requires='>=3'
     )