# Reproducible build + test image for the physics sandbox.
#   docker build -t physics-sandbox .
#   docker run --rm physics-sandbox                 # runs the test harness
#   docker run --rm physics-sandbox ./build/sandbox --scenario bounce --steps 400
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake libbullet-dev python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j

# Default: run the native tests + Python regression harness.
CMD ["sh", "-c", "ctest --test-dir build --output-on-failure && python3 scripts/run_tests.py"]
