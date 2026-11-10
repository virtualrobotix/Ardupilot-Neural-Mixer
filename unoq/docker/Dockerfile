# Build and bench-test image for the Arduino Uno Q target (Debian arm64, same architecture as the board).
FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential g++ git ca-certificates python3 python3-pip rsync \
    && rm -rf /var/lib/apt/lists/*
# waf code generators (empy 3 is required by the ArduPilot build) and the MAVLink test client
RUN pip3 install --break-system-packages --no-cache-dir \
        empy==3.3.4 pexpect future setuptools pymavlink
RUN git config --system --add safe.directory '*'
WORKDIR /work/ardupilot
