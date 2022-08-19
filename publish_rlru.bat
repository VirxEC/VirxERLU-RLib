@echo off
"%localappdata%\RLBotGUIX\Python37\python.exe" -m pip install -U maturin twine
maturin publish --release -i "%localappdata%\RLBotGUIX\Python37\python.exe"
pause