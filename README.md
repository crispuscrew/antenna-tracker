# antenna-tracker

Автономное C++17-приложение для сопровождения спутников через контроллер антенны.
Контроллер подключается по TCP (RS-432 → Moxa 5200 → TCP).
Приложение работает без GUI, конфигурируется через аргументы командной строки
и запускается из внешнего скрипта (враппер на Python/Go).

---

## Зависимости

| Библиотека | Версия | Назначение | Получение |
|---|---|---|---|
| [Asio](https://github.com/chriskohlhoff/asio) | 1.30.2 | TCP + таймеры (standalone, без Boost) | FetchContent |
| [libpredict](https://github.com/la1k/libpredict) | 2.0.0 | SGP4 / расчёт AOS-LOS и положения спутника | FetchContent |

Системных зависимостей нет. Всё скачивается при первом `cmake ..`.

---

## Сборка

### Через Makefile (рекомендуется)

```bash
make              # сборка Release
make test         # unit-тесты
make rebuild      # чистая пересборка
make mock         # запустить mock-контроллер
```

### Напрямую через CMake

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
```

### Контейнер (Docker / Podman)

Makefile автоматически определяет доступный runtime (podman приоритетнее docker):

```bash
make docker                        # сборка образа
make docker-run ARGS="--ip 192.168.1.100 --port 4001 \
                      --lat 55.7 --lon 37.6 --alt 200 \
                      --tle /data/sat.tle"

# Принудительно использовать конкретный runtime:
make docker CONTAINER_RT=docker
make docker CONTAINER_RT=podman
```

> **Важно:** перед `make docker` убедитесь, что `build/` либо не существует,
> либо исключена через `.dockerignore` (уже включён в репо). Локальный
> `CMakeCache.txt` содержит абсолютные пути хоста и сломает сборку внутри
> контейнера, если попадёт в контекст.

Бинарник: `build/antenna-tracker`

---

## Запуск

```bash
./antenna-tracker \
    --ip   192.168.1.100 \
    --port 4001          \
    --lat  55.7          \
    --lon  37.6          \
    --alt  200           \
    --tle  /path/to/sat.tle \
    [--debug]
```

### Аргументы

| Аргумент | Обязателен | По умолчанию | Описание |
|---|---|---|---|
| `--ip <адрес>` | да | — | IP-адрес контроллера антенны |
| `--port <порт>` | нет | `4001` | TCP-порт |
| `--lat <градусы>` | да | — | Широта наземной станции (десятичные градусы) |
| `--lon <градусы>` | да | — | Долгота наземной станции (десятичные градусы) |
| `--alt <метры>` | нет | `0` | Высота над уровнем моря, метры |
| `--tle <файл>` | да | — | Путь к файлу TLE (2 или 3 строки) |
| `--debug` | нет | выкл | Подробный вывод: az/el каждые 100 мс, AOS/LOS |

### Формат TLE-файла

Принимаются как двухстрочные TLE (без имени), так и трёхстрочные (с именем):

```
ISS (ZARYA)
1 25544U 98067A   24001.50000000  .00006421  00000-0  12100-3 0  9993
2 25544  51.6400 123.4567 0003456  78.9012 281.1234 15.50000000 12345
```

### Тест без реального контроллера

```bash
# Терминал 1 — фиктивный контроллер
nc -l 4001 | xxd

# Терминал 2 — трекер
./antenna-tracker --ip 127.0.0.1 --port 4001 \
                  --lat 55.7 --lon 37.6 --alt 200 \
                  --tle ../sat.tle --debug
```

---

## Архитектура

```
antenna-tracker/
├── CMakeLists.txt
└── src/
    ├── main.cpp                        # Разбор аргументов, запуск io_context
    ├── config.h                        # POD-структура Config
    ├── protocol/
    │   ├── drive.h                     # Drive (состояние антенны), DriveCmd (команды)
    │   ├── vkaprotocol.h               # Интерфейс кодирования/декодирования пакетов
    │   └── vkaprotocol.cpp             # Реализация (snprintf, без Qt)
    ├── tracking/
    │   ├── tle_tracker.h/.cpp          # Обёртка над libpredict: compute(), next_aos(), next_los()
    │   └── trajectory_planner.h/.cpp   # Предрасчёт траектории, раскрутка азимута
    └── controller/
        ├── antenna_controller.h
        └── antenna_controller.cpp      # Asio TCP, 10 Гц таймер, машина состояний
```

### Машина состояний

```
WAITING ──(AOS − 60 с)──► ACQUIRING ──(AOS)──► TRACKING ──(LOS)──► STOPPING ──► WAITING
                                                                          │
                                                              отправить cmd=03 (STOP)
```

| Состояние | Действие |
|---|---|
| `WAITING` | Вычисляет следующий проход (AOS/LOS), ждёт момента `AOS − 60 с` |
| `ACQUIRING` | Каждые 100 мс посылает команду первой точки траектории (преднаведение) |
| `TRACKING` | Каждые 100 мс берёт az/el из предрасчитанной траектории и посылает команду `TRACKER` |
| `STOPPING` | Посылает команду `STOP` (cmd=03), переходит в `WAITING` |

Весь event loop однопоточный — один `asio::io_context`.

---

## Протокол VKA

### Формат команды (30 байт)

```
 Байты  │ Поле     │ Пример
────────┼──────────┼──────────────────────────────
  0–3   │ Header   │ 5555
  4–5   │ seq      │ 01           (0–99, decimal)
  6–7   │ cmd      │ 02           (02=TRACKER, 03=STOP)
  8–13  │ data1    │ 000000       (зарезервировано)
 14–19  │ az       │ +05400       (±DDDCC = 54.00°)
 20–25  │ el       │ +04500       (45.00°)
 26–27  │ CRC      │ A3           (LRC, uppercase hex)
 28–29  │ Term     │ \r\n
```

### Формат ответа (30 байт)

```
 Байты  │ Поле     │ Описание
────────┼──────────┼──────────────────────────────
  0–3   │ Header   │ 5555
  4–5   │ seq      │ порядковый номер
  6–9   │ state    │ состояние контроллера
 10–15  │ az       │ текущий азимут (±DDDCC)
 16–21  │ el       │ текущее угол места (±DDDCC)
 22–25  │ reserve  │ зарезервировано
 26–27  │ CRC      │ LRC
 28–29  │ Term     │ \r\n
```

**Формат угла `±DDDCC`**: знак + 3 цифры целой части + 2 цифры сотых.
Например: `+05400` = 54.00°, `-00175` = −1.75°.

**CRC (LRC)**: `(256 − (сумма байт % 256)) % 256` по байтам 4–25 (seq…el/reserve),
результат — 2 символа uppercase hex.

---

## Алгоритм раскрутки азимута

Перед каждым проходом предрасчитывается полная траектория с шагом 1 с.
Чтобы антенна не делала оборот через 0°/360°, каждая точка корректируется:

```
offsets[] = {0, +360, −360, −720}

первая точка:
    выбрать offset такой, чтобы raw_az + offset ∈ [−250, +250]

каждая последующая точка:
    выбрать offset, при котором |raw_az + offset − prev_az| минимален
    commanded_az = raw_az + offset
```

Клампить промежуточные точки нельзя — это разрывает непрерывность
траектории и вызывает резкий скачок антенны.

Во время сопровождения выбирается точка с ближайшим меткой времени
(интерполяция не нужна: шаг 1 с, частота управления 10 Гц).

---

## Тестирование

### Unit-тесты (C++, без сети и железа)

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc) test-protocol test-trajectory

# Запустить все тесты через CTest
ctest -V

# Или напрямую
./test-protocol
./test-trajectory
```

Что покрыто:

| Тест | Что проверяется |
|---|---|
| `test-protocol` | `compute_lrc`, `format_angle`, `parse_angle`, roundtrip, `pack`/`unpack`, CRC-ошибки |
| `test-trajectory` | `lookup()` (пустая, одна точка, точное совпадение, ближайший, clamp), алгоритм раскрутки азимута |

---

### Mock-контроллер (интеграционный тест без железа)

`tools/mock_controller.py` имитирует TCP-интерфейс контроллера: принимает
30-байтовые команды, печатает декодированный az/el, отвечает фиктивным
пакетом состояния.

**Терминал 1 — mock:**
```bash
python3 tools/mock_controller.py --port 4001 --noise 0.1
```

**Терминал 2 — трекер:**
```bash
./build/antenna-tracker \
    --ip 127.0.0.1 --port 4001 \
    --lat 55.7 --lon 37.6 --alt 200 \
    --tle sat.tle --debug
```

Пример вывода mock:
```
[mock] Ожидание подключений на порту 4001 (шум=0.1°) …
[+] Подключение: ('127.0.0.1', 54321)
  [TRACKER] seq=00  az=  +54.00°  el= +30.00°
  [TRACKER] seq=01  az=  +54.12°  el= +30.05°
  ...
  [STOP   ] seq=42  az=   +0.00°  el=  +0.00°
```

Опции mock:

| Флаг | По умолчанию | Описание |
|---|---|---|
| `--port` | `4001` | TCP-порт |
| `--noise` | `0.0` | СКО гауссовского шума к текущему положению (градусы) |

---

## Исходные данные libpredict

| Функция | Описание |
|---|---|
| `predict_parse_tle(l1, l2)` | Разобрать TLE, вернуть `predict_orbital_elements_t*` |
| `predict_create_observer(name, lat_rad, lon_rad, alt_m)` | Создать наблюдателя |
| `predict_orbit(sat, &pos, jd)` | Вычислить орбитальное положение |
| `predict_observe_orbit(obs, &pos, &obs_data)` | Вычислить az/el (в радианах) |
| `predict_next_aos(obs, sat, jd)` | Следующий AOS → `predict_observation.time` |
| `predict_next_los(obs, sat, jd)` | Следующий LOS → `predict_observation.time` |
| `predict_to_julian(time_t)` | Unix-время → libpredict Julian date |

`predict_julian_date_t` — дни с 31 декабря 1979 00:00:00 UTC (не стандартная JD).
