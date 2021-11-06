import os

from setuptools import setup
from setuptools_rust import Binding, RustExtension

# pip install -U setuptools wheel setuptools-rust twine
# python setup.py sdist bdist_wheel
# twine upload dist/*

long_description = ""
with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), 'README.md'), "r") as f:
    long_description = f.read()

setup(
    name="VirxERLU-RLib",
    version="0.5.0",
    description='Rust modules for VirxERLU',
    long_description=long_description,
    rust_extensions=[RustExtension("virxrlru", binding=Binding.RustCPython)],
    license="MIT",
    author='VirxEC',
    author_email='virx@virxcase.dev',
    url="https://github.com/VirxEC/VirxERLU-RLib",
    python_requires='>=3.7',
    # Rust extensions are not zip safe
    zip_safe=False,
)
