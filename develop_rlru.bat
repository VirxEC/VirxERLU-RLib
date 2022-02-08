@echo off
set rlpy="%localappdata%\RLBotGUIX\Python37\python.exe"
cargo fmt
maturin build --release -i %rlpy%
%rlpy% -m pip install target\wheels\VirxERLU_RLib-*.*.*-cp37-*.whl --force-reinstall