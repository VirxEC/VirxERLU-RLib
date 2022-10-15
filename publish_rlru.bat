@echo off
"%localappdata%\RLBotGUIX\Python37\python.exe" -m pip install -U maturin twine
maturin publish -i "%localappdata%\RLBotGUIX\Python37\python.exe"
pause