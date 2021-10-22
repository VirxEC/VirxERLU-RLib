@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
cargo fmt
%rlpy% setup.py install
%rlpy% test.py
pause