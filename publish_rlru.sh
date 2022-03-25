set +v
source $HOME/.cargo/env
source venv37/bin/activate
maturin build --release -i $(which python)
source venv39/bin/activate
maturin build --release -i $(which python)
twine upload -r pypi target/wheels/*
