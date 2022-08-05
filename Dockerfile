# syntax=docker/dockerfile:1
FROM quay.io/pypa/manylinux_2_24_x86_64
SHELL [ "/bin/bash", "-c" ]

WORKDIR /rlib

COPY . .

RUN chmod +x ./publish_rlru.sh && \
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
source $HOME/.cargo/env && \
/opt/python/cp37-cp37m/bin/python -m venv venv37 && \
source venv37/bin/activate && \
pip install maturin twine && \
mv .pypirc ~/.pypirc

ENTRYPOINT [ "/bin/bash", "-c", "./publish_rlru.sh" ]
