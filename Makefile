.DEFAULT_GOAL := all

BUILD_DIR  := build
BUILD_TYPE ?= Release
JOBS       := $(shell nproc)

# ── Container runtime (podman предпочтительнее, fallback на docker) ───────────
# Переопределить: make docker CONTAINER_RT=docker
CONTAINER_RT ?= $(shell command -v podman 2>/dev/null \
                    || command -v docker 2>/dev/null \
                    || echo "")

# Короткое имя для логов (podman / docker / <пусто>)
CONTAINER_RT_NAME := $(notdir $(CONTAINER_RT))

IMAGE_NAME ?= antenna-tracker

_check_container_rt:
	@test -n "$(CONTAINER_RT)" || \
	    (echo "Ошибка: не найден ни podman, ни docker. Установите один из них."; exit 1)
	@echo "[container] Используется: $(CONTAINER_RT_NAME) ($(CONTAINER_RT))"

.PHONY: all build test clean rebuild docker docker-run mock help \
        _check_container_rt

## Сборка (по умолчанию)
all: build

## Конфигурация + сборка
build: $(BUILD_DIR)/Makefile
	cmake --build $(BUILD_DIR) --parallel $(JOBS)

$(BUILD_DIR)/Makefile:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

## Пересборка с нуля
rebuild: clean build

## Unit-тесты через CTest
test: build
	cd $(BUILD_DIR) && ctest --output-on-failure -j$(JOBS)

## Удалить build-директорию
clean:
	rm -rf $(BUILD_DIR)

## Сборка образа контейнера (podman или docker)
docker: _check_container_rt
	$(CONTAINER_RT) build -t $(IMAGE_NAME) .

## Запуск контейнера
## Пример: make docker-run ARGS="--ip 192.168.1.100 --port 4001 --lat 55.7 --lon 37.6 --tle /data/sat.tle"
docker-run: _check_container_rt
	$(CONTAINER_RT) run --rm $(IMAGE_NAME) $(ARGS)

## Запустить mock-контроллер (нужен Python 3)
mock:
	python3 tools/mock_controller.py --port 4001 $(MOCK_ARGS)

help:
	@echo "Цели:"
	@echo "  make              — сборка (Release)"
	@echo "  make test         — unit-тесты"
	@echo "  make rebuild      — чистая пересборка"
	@echo "  make clean        — удалить build/"
	@echo "  make docker       — собрать образ (podman/docker)"
	@echo "  make docker-run   — запустить в контейнере"
	@echo "  make mock         — mock-контроллер на порту 4001"
	@echo ""
	@echo "Переменные:"
	@echo "  BUILD_TYPE=Debug|Release      (по умолчанию Release)"
	@echo "  CONTAINER_RT=/path/to/podman  (по умолчанию: auto-detect)"
	@echo "  IMAGE_NAME=antenna-tracker    (имя образа)"
	@echo "  ARGS='--ip ...'               — аргументы для docker-run"
	@echo "  MOCK_ARGS='--noise 0.2'       — аргументы для mock_controller.py"
	@echo ""
	@echo "Примеры:"
	@echo "  make docker CONTAINER_RT=docker"
	@echo "  make docker-run ARGS='--ip 127.0.0.1 --port 4001 --lat 55.7 --lon 37.6 --tle /data/sat.tle'"
