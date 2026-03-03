#!/usr/bin/env python3
"""
mock_controller.py — имитация TCP-интерфейса контроллера антенны VKA.

Принимает входящие 30-байтовые команды от antenna-tracker,
декодирует их, печатает az/el и отвечает фиктивным пакетом состояния.
Текущее положение антенны «следует» за командой с опциональным шумом.

Запуск:
    python3 tools/mock_controller.py [--port 4001] [--noise 0.1]
"""

import socket
import argparse
import random
import math
import sys

PACKET_LEN = 30


# ── Протокол ──────────────────────────────────────────────────────────────────

def compute_lrc(data: bytes) -> int:
    """LRC = (256 − (сумма байт % 256)) % 256"""
    return (256 - (sum(data) % 256)) % 256


def format_angle(deg: float) -> bytes:
    """Кодирует угол в формат ±DDDCC (6 байт)."""
    sign = b"+" if deg >= 0 else b"-"
    deg = abs(deg)
    whole = int(deg)
    cents = round((deg - whole) * 100)
    if cents >= 100:
        cents -= 100
        whole += 1
    return sign + f"{whole:03d}{cents:02d}".encode()


def parse_angle(s: bytes) -> float:
    """Декодирует угол из формата ±DDDCC."""
    sign = 1.0 if s[0:1] == b"+" else -1.0
    whole = int(s[1:4])
    cents = int(s[4:6])
    return sign * (whole + cents / 100.0)


def make_response(seq: int, az: float, el: float) -> bytes:
    """
    Строит 30-байтовый ответный пакет контроллера.
    Layout: 5555 | seq(2) | state(4) | az(6) | el(6) | reserve(4) | crc(2) | \\r\\n
    """
    payload = (
        f"{seq % 100:02d}".encode()   # seq
        + b"0001"                      # state = "готов"
        + format_angle(az)             # текущий азимут
        + format_angle(el)             # текущий угол места
        + b"0000"                      # reserve
    )
    crc = compute_lrc(payload)
    return b"5555" + payload + f"{crc:02X}".encode() + b"\r\n"


def parse_command(pkt: bytes) -> dict | None:
    """
    Разбирает 30-байтовую команду.
    Layout: 5555 | seq(2) | cmd(2) | data1(6) | az(6) | el(6) | crc(2) | \\r\\n
    Возвращает dict или None при ошибке.
    """
    if len(pkt) < PACKET_LEN:
        return None
    if pkt[0:4] != b"5555":
        return None
    if pkt[28:30] != b"\r\n":
        return None

    payload = pkt[4:26]
    crc_expected = compute_lrc(payload)
    crc_received = int(pkt[26:28], 16)
    if crc_expected != crc_received:
        print(f"  [!] CRC: ожидался {crc_expected:02X}, получен {crc_received:02X}",
              flush=True)
        return None

    return {
        "seq": int(pkt[4:6]),
        "cmd": int(pkt[6:8]),
        "az":  parse_angle(pkt[14:20]),
        "el":  parse_angle(pkt[20:26]),
    }


# ── Сервер ────────────────────────────────────────────────────────────────────

CMD_NAMES = {2: "TRACKER", 3: "STOP"}


def handle_connection(conn: socket.socket, addr, noise: float) -> None:
    print(f"[+] Подключение: {addr}", flush=True)
    buf = b""
    current_az = 0.0
    current_el = 0.0
    resp_seq = 0
    pkt_count = 0

    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk

            while len(buf) >= PACKET_LEN:
                # Найти заголовок
                pos = buf.find(b"5555")
                if pos < 0:
                    buf = b""
                    break
                if pos > 0:
                    print(f"  [!] Пропуск {pos} байт мусора перед заголовком",
                          flush=True)
                    buf = buf[pos:]
                if len(buf) < PACKET_LEN:
                    break

                cmd = parse_command(buf[:PACKET_LEN])
                buf = buf[PACKET_LEN:]

                if cmd is None:
                    continue

                name = CMD_NAMES.get(cmd["cmd"], f"cmd={cmd['cmd']}")
                pkt_count += 1
                print(
                    f"  [{name:7s}] seq={cmd['seq']:02d}  "
                    f"az={cmd['az']:+8.2f}°  el={cmd['el']:+7.2f}°",
                    flush=True,
                )

                # Антенна «следует» за командой (с шумом)
                if cmd["cmd"] == 2:  # TRACKER
                    current_az = cmd["az"] + random.gauss(0, noise)
                    current_el = cmd["el"] + random.gauss(0, noise)
                elif cmd["cmd"] == 3:  # STOP
                    current_az = 0.0
                    current_el = 0.0

                resp = make_response(resp_seq, current_az, current_el)
                resp_seq = (resp_seq + 1) % 100
                conn.sendall(resp)

    except (ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        conn.close()
        print(f"[-] Отключение: {addr}  (принято пакетов: {pkt_count})", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mock VKA antenna controller для тестирования antenna-tracker"
    )
    parser.add_argument("--port",  type=int,   default=4001,
                        help="TCP-порт (по умолчанию 4001)")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="СКО гауссовского шума к текущему az/el (градусы)")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(("0.0.0.0", args.port))
    except OSError as e:
        print(f"Ошибка bind на порту {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    server.listen(1)
    print(f"[mock] Ожидание подключений на порту {args.port} "
          f"(шум={args.noise}°) …", flush=True)
    print( "[mock] Ctrl+C для выхода\n", flush=True)

    try:
        while True:
            conn, addr = server.accept()
            handle_connection(conn, addr, args.noise)
    except KeyboardInterrupt:
        print("\n[mock] Завершение.", flush=True)
    finally:
        server.close()


if __name__ == "__main__":
    main()
