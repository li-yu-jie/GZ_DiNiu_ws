import struct, os, select, time, sys

dev = '/dev/input/js0'
fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
print(f"监听 {dev} 10 秒——请用力晃两个摇杆...", flush=True)
t0 = time.time()
n_axis = n_btn = 0
axis_seen = {}
while time.time() - t0 < 10:
    rl, _, _ = select.select([fd], [], [], 0.5)
    if not rl:
        continue
    data = os.read(fd, 8 * 64)
    for off in range(0, len(data) - 7, 8):
        t_ms, val, etype, num = struct.unpack('IhBB', data[off:off + 8])
        base = etype & ~0x80
        if base == 2:
            n_axis += 1
            axis_seen[num] = val
        elif base == 1:
            n_btn += 1
            print(f"  按钮事件: 号={num} 值={val}", flush=True)
print(f"轴事件总数={n_axis}  按钮事件数={n_btn}", flush=True)
print(f"出现过的轴: {axis_seen}", flush=True)
