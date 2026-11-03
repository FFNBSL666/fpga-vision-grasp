"""Camera capture/transport only. All resizing and inference belong on FPGA.

P1/UART1_TX -> BASEBOARD J3 pin1 (K14), GND -> J3 pin37.
Independent camera USB power. J3 pin38 is +3.3V, NOT GND.
921600 baud, 8N1; use model_diagnostic_locked.bit for the current model test.
The historical camera_link_locked.bit accepts the same RGB565 wire protocol.
"""
import sensor
import time
from pyb import UART
import machine
import os
import sys
import gc

WIDTH = 160
HEIGHT = 120
BAUD = 921600
# Set True for a known red/green/blue/white pattern during the first wiring test.
TEST_PATTERN = False


def make_crc_table():
    table = []
    for value in range(256):
        crc = value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
        table.append(crc)
    return table


CRC_TABLE = make_crc_table()


def crc_update(crc, data):
    for value in data:
        crc = ((crc << 8) ^ CRC_TABLE[((crc >> 8) ^ value) & 255]) & 0xFFFF
    return crc


def write_all(uart, data):
    view = memoryview(data)
    offset = 0
    while offset < len(view):
        count = uart.write(view[offset:])
        if not count:
            raise OSError("UART write timeout")
        offset += count


def send_frame(uart, img, sequence):
    header = bytes((2, sequence & 255, WIDTH, HEIGHT, 1))
    write_all(uart, b"\xA5\x5A" + header)
    crc = crc_update(0xFFFF, header)
    row = bytearray(WIDTH * 2)
    colors = (0xF800, 0x07E0, 0x001F, 0xFFFF)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            # Packing native RGB565 is transport formatting, no model/resize here.
            value = colors[x // 40] if TEST_PATTERN else img.get_pixel(x, y, rgbtuple=False)
            row[2*x] = value & 255
            row[2*x+1] = (value >> 8) & 255
        crc = crc_update(crc, row)
        write_all(uart, row)
    write_all(uart, bytes((crc >> 8, crc & 255)))



# Recovery only: capture/UART packet format and FPGA inference are unchanged.
WATCHDOG_MS = 8000
EVENT_LOG = "camera_sender_events.log"


def log_event(event, detail):
    line = "%d %s %s\n" % (time.ticks_ms(), event, detail)
    print("CAMERA_SENDER", line.strip())
    try:
        try:
            if os.stat(EVENT_LOG)[6] > 4096:
                try:
                    os.remove(EVENT_LOG + ".previous")
                except OSError:
                    pass
                os.rename(EVENT_LOG, EVENT_LOG + ".previous")
        except OSError:
            pass
        with open(EVENT_LOG, "a") as stream:
            stream.write(line)
        if hasattr(os, "sync"):
            os.sync()
    except Exception:
        # A missing/full log volume must not stop capture and transport.
        pass


def main():
    watchdog = machine.WDT(timeout=WATCHDOG_MS)
    log_event("boot", "reset_cause=%d watchdog_ms=%d" % (machine.reset_cause(), WATCHDOG_MS))
    sequence = 0
    failures = 0
    while True:
        uart = None
        stage = "sensor_init"
        try:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QQVGA)
            sensor.skip_frames(time=2000)
            stage = "uart_init"
            uart = UART(1, BAUD, bits=8, parity=None, stop=1, timeout=1000, timeout_char=1000)
            first_frame = True
            while True:
                stage = "snapshot"
                img = sensor.snapshot()
                if img.width() != WIDTH or img.height() != HEIGHT:
                    raise RuntimeError("Expected RGB565 QQVGA 160x120")
                stage = "uart_send"
                send_frame(uart, img, sequence)
                sequence = (sequence + 1) & 255
                failures = 0
                # Feed only after one entire valid image packet was sent.
                watchdog.feed()
                if first_frame:
                    log_event("stream_started", "sequence=%d" % sequence)
                    first_frame = False
                time.sleep_ms(20)
        except Exception as error:
            failures += 1
            log_event("retry", "stage=%s count=%d error=%s" % (stage, failures, repr(error)))
            if uart is not None:
                try:
                    uart.deinit()
                except Exception:
                    pass
            gc.collect()
            if failures >= 3:
                log_event("reset", "repeated_capture_or_uart_failure")
                machine.reset()
            time.sleep_ms(300)


if __name__ == "__main__":
    try:
        main()
    except BaseException as error:
        log_event("script_exit", repr(error))
        # STOP/soft reset cannot feed the watchdog. It performs a hardware
        # reset, after which the saved main.py automatically starts again.
        raise
