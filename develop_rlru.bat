@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
%rlpy% -m pip install -U setuptools wheel setuptools_rust --no-warn-script-location
%rlpy% setup.py install
pause