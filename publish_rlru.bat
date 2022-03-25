@echo off
cargo update
cargo fmt
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
%rlpy% -m pip install -U maturin --no-warn-script-location
maturin build --release -i %rlpy%
%rlpy% -m twine upload target\wheels\*.whl
pause