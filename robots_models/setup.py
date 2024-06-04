from setuptools import setup

setup(name='robots_models',
        version='0.1',
        description='Custom robots models for Python Robotics Toolbox',
        url='#',
        author='Juan Helios García Guzmán',
        author_email='jhelg@ugr.es',
        license='MIT',
        packages=['robots_models'],
        install_requires=["roboticstoolbox-python>=1.0.2"],
        zip_safe=False,
        include_package_data=True,
        package_data={'': ['data/*.xacro']},
)