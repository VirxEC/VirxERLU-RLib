@echo off
python -m pip install -U maturin twine
maturin build --release -i "%localappdata%\RLBotGUIX\Python37\python.exe"
maturin build --release -i "%localappdata%\RLBotGUIX\Python38\python.exe"
maturin build --release -i "%localappdata%\RLBotGUIX\Python39\python.exe"
twine upload target\wheels\*.whl
pause