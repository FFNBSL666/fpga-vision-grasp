"""Independent Python golden vectors for full 160x120 ->96x96 RTL checking."""
from pathlib import Path
import binascii
import importlib.util
import sys
import types

ROOT = Path(__file__).resolve().parents[1]


def resize_reference(src):
    import numpy as np
    src = np.asarray(src, dtype=np.int64)
    h, w = src.shape[:2]
    result = np.empty((96, 96, 3), dtype=np.uint8)
    for y in range(96):
        qy = (((2*y+1)*h-96)*256)//192
        sy, fy = divmod(qy, 256)
        for x in range(96):
            qx = (((2*x+1)*w-96)*256)//192
            sx, fx = divmod(qx, 256)
            top = src[sy, sx]*(256-fx) + src[sy, sx+1]*fx
            bottom = src[sy+1, sx]*(256-fx) + src[sy+1, sx+1]*fx
            result[y, x] = (top*(256-fy)+bottom*fy+32768)//65536
    return result


def rgb565_roundtrip(src):
    import numpy as np
    v = np.asarray(src, dtype=np.uint16)
    packed = ((v[:, :, 0]>>3)<<11)|((v[:, :, 1]>>2)<<5)|(v[:, :, 2]>>3)
    r, g, b = (packed>>11)&31, (packed>>5)&63, packed&31
    expanded = np.stack(((r<<3)|(r>>2), (g<<2)|(g>>4), (b<<3)|(b>>2)), axis=2).astype('uint8')
    return packed, expanded


def main():
    import numpy as np
    out = ROOT/'work'/'camera_vectors'
    out.mkdir(parents=True, exist_ok=True)
    # Independent PRNG, nonuniform pattern exercises channels, borders and carries.
    rng = np.random.default_rng(20261024)
    image = rng.integers(0, 256, size=(120, 160, 3), dtype=np.uint8)
    packed, expanded = rgb565_roundtrip(image)
    header = bytes((2, 37, 160, 120, 1))
    payload = packed.astype('<u2').tobytes()
    crc = binascii.crc_hqx(header+payload, 0xffff)
    packet = b'\xa5\x5a'+header+payload+crc.to_bytes(2,'big')
    (out/'packet.hex').write_text(''.join(f'{v:02x}\n' for v in packet), encoding='ascii')
    expected = resize_reference(expanded)
    colors = (expected[:, :, 0].astype('uint32')<<16)|(expected[:, :, 1].astype('uint32')<<8)|expected[:, :, 2]
    (out/'expected.hex').write_text(''.join(f'{v:06x}\n' for v in colors.flat), encoding='ascii')
    # Run the actual MicroPython sender with small mocks; compare every wire byte.
    sys.modules['sensor'] = types.ModuleType('sensor')
    pyb = types.ModuleType('pyb'); pyb.UART = object; sys.modules['pyb'] = pyb
    spec = importlib.util.spec_from_file_location('camera_sender', ROOT/'camera'/'openmv_uart_rgb565_160x120.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    class FakeImage:
        def get_pixel(self, x, y, rgbtuple=False): return int(packed[y,x])
    class PartialUART:
        def __init__(self): self.buffer = bytearray()
        def write(self, view):
            size = min(len(view), 71); self.buffer.extend(view[:size]); return size
    uart = PartialUART(); module.send_frame(uart, FakeImage(), 37)
    assert bytes(uart.buffer) == packet, 'Camera sender wire bytes differ'
    assert module.crc_update(0xffff,b'123456789') == 0x29B1
    print(f'PASS: CAMERA_SENDER {len(packet)} bytes; CRC16 known-answer; partial writes; golden pixels=9216')


if __name__ == '__main__': main()
