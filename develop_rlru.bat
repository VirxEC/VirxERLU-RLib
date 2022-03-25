@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
cargo fmt
maturin build --release -i %rlpy%
%rlpy% -m pip install VirxERLU-RLib --find-links=target\wheels --force-reinstall