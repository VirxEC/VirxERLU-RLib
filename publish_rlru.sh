set +v
source $HOME/.cargo/env
source venv37/bin/activate
maturin build --release
twine upload -r pypi target/wheels/*
