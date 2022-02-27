set +v
cargo fmt
maturin build --release -i "/usr/bin/python3.7"
source ~/.RLBotGUI/env/bin/activate
pip install target/wheels/VirxERLU_RLib-*.*.*-cp37-*.whl --force-reinstall