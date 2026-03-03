# Совместим с Docker и Podman (podman build / docker build).
# Требует: .dockerignore исключает build/ чтобы локальный CMakeCache не попал в контекст.

FROM debian:bookworm-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel $(nproc)

# ── Минимальный runtime-образ ─────────────────────────────────────────────────
FROM debian:bookworm-slim AS runtime

COPY --from=builder /src/build/antenna-tracker /usr/local/bin/antenna-tracker

ENTRYPOINT ["antenna-tracker"]
