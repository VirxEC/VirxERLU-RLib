@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
cargo fmt
%rlpy% -m pip install .
%rlpy% test.py
pause