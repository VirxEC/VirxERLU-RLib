@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
cargo fmt
maturin build --release -i %rlpy%
%rlpy% -m pip install target\wheels\VirxERLU_RLib-0.8.0-cp37-none-win_amd64.whl --force-reinstall