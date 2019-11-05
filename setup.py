from setuptools import setup, find_packages

long_description = '''
=======================================================================
wire_skin: construct a simple skin around a wire frame mesh in Blender.
=======================================================================

Please visit the Github repository for documentation:

`<https://github.com/cwant/wire_skin>`_

Either checkout the code from the Github project, or install via pip::

    python3 -m pip install wire_skin

or::

    pip3 install wire_skin

'''[1:-1]

setup(
    name='wire_skin',
    version='0.3',
    description='Construct a simple skin around a wire frame mesh in Blender.',
    long_description=long_description,
    url='https://github.com/cwant/wire_skin',
    author='Chris Want',
    classifiers=['Development Status :: 3 - Alpha',
                 'Intended Audience :: Developers',
                 'Intended Audience :: Manufacturing',
                 'Intended Audience :: Science/Research',
                 'License :: OSI Approved :: Apache Software License',
                 'Natural Language :: English',
                 'Programming Language :: Python :: 3 :: Only',
                 'Topic :: Artistic Software',
                 'Topic :: Multimedia :: Graphics :: 3D Modeling',
                 'Topic :: Scientific/Engineering :: Mathematics',
                 'Topic :: Scientific/Engineering :: Visualization'],
    keywords='skin wire frame modeling blender',
    packages=find_packages(exclude=['tests', 'demo']),
    python_requires='~=3.5'
)
