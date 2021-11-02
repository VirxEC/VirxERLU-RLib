#!/bin/zsh

cargo fmt
source $HOME/.RLBotGUI/env/bin/activate
python -m pip install .
python test.py