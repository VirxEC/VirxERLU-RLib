#!/bin/zsh

cargo update
source ~/.RLBotGUI/env/bin/activate
python -m pip install -U setuptools wheel setuptools-rust twine --no-warn-script-location
python setup.py sdist bdist_wheel
python -m twine upload dist/*