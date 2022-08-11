set +v
cargo fmt
maturin build --release
source ~/.RLBotGUI/env/bin/activate
pip install target/wheels/VirxERLU_RLib-*.*.*-cp37-abi3-*.whl --force-reinstall