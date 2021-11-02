@echo off
cargo update
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
%rlpy% -m pip install -U setuptools wheel setuptools-rust twine --no-warn-script-location
%rlpy% setup.py sdist bdist_wheel
%rlpy% -m twine upload dist/*
pause