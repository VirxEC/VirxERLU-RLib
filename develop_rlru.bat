@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
%rlpy% setup.py install
%rlpy% test.py
pause