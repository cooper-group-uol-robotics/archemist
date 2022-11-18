#!/usr/bin/env python3

from setuptools import setup

setup(
     name='archemist',
     version='0.3',
     author='stoic-roboticist',
     author_email='hatem.fakhruldeen@gmail.com',
     url='https://github.com/cooper-group-uol-robotics/archemist',
     packages=['archemist'],
     package_dir={'':'src'},
     install_requires=[  'mongoengine==0.24.2',
                         'multipledispatch==0.6.0',
                         'pyzmq==23.2.1',
                         'transitions==0.6.4',
                         'multipledispatch==0.6.0',
                         'PyInquirer==1.0.3',
                         'watchdog==2.1.9',
                         'strictyaml==1.6.2',],
     scripts=[ 'scripts/run_server',
               'scripts/run_cli',
               'scripts/create_workflow_dir',
               'scripts/launch_handlers',
               'scripts/clear_db']
)