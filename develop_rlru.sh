set +v
cargo fmt
maturin build --release -i "/usr/bin/python3.7"
python3.7 -m pip install target/wheels/VirxERLU_RLib-*.*.*-cp37-*.whl --force-reinstall