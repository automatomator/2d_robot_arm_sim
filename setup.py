# setup.py
from setuptools import setup, find_packages

setup(
    name='rob_arm_simulator',
    version='0.1.0', # Initial version number
    author='Your Name', # Replace with your name
    author_email='your.email@example.com', # Replace with your email
    description='A 2-DOF Robotic Arm Simulator with GUI and Animation',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/rob_arm_simulator', # Replace with your GitHub repo URL if applicable
    packages=find_packages(), # Automatically finds packages in your project (e.g., rob_arm_sim)
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License', # Assuming MIT license, see section below
        'Operating System :: OS Independent',
        'Development Status :: 3 - Alpha', # Or 4 - Beta, 5 - Production/Stable
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Robotics',
        'Topic :: Scientific/Engineering :: Physics',
        'Topic :: Software Development :: User Interfaces',
    ],
    python_requires='>=3.8', # Minimum Python version required
    install_requires=[
        'numpy>=1.20',
        'matplotlib>=3.3',
        'PyQt5>=5.15',
    ],
    entry_points={
        'gui_scripts': [
            'rob_arm_sim_gui=gui_app:main', # Allows running the GUI via a command
        ],
    },
)