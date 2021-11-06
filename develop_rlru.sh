#!/bin/zsh

cargo fmt
source ~/.RLBotGUI/env/bin/activate
python -m pip install .
python test.py