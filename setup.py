from setuptools import setup, find_packages

setup(
    name='wave_rover_serial',
    version='0.1',
    packages=find_packages(),
    description='A package to simplify the wave rover serial communication',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='Maxime Santerre',
    author_email='maxime.santerre@chime.com',
    url='https://github.com/msanterre/wave_rover_serial',
    install_requires=[
        'pyserial>=3.4',
    ],
    python_requires='>=3.8',
)
