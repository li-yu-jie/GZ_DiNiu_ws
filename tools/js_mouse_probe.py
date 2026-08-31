import struct, os, select, time

# BFM 接收器还暴露了一个 Mouse 设备（event26 / mouse4）
# 若摇杆被切到鼠标模式，晃摇杆时这里会出现 REL_X/REL_Y (type=2) 事件
dev = '/dev/input/event26'
fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
print(f"监听 {dev} 10 秒——请继续晃两个摇杆...", flush=True)
t0 = time.time()
events = {}
while time.time() - t0 < 10:
    rl, _, _ = select.select([fd], [], [], 0.5)
    if not rl:
        continue
    data = os.read(fd, 24 * 64)
    for off in range(0, len(data) - 23, 24):
        sec, usec, etype, code, val = struct.unpack('QQHHi', data[off:off + 24])
        key = (etype, code)
        events[key] = events.get(key, 0) + 1
if events:
    names = {(2, 0): 'REL_X', (2, 1): 'REL_Y', (2, 8): 'REL_WHEEL',
             (1, 0x110): 'BTN_LEFT', (1, 0x111): 'BTN_RIGHT', (0, 0): 'SYN'}
    for k, v in sorted(events.items()):
        print(f"  {names.get(k, f'type={k[0]} code={k[1]}')}: {v} 个事件", flush=True)
else:
    print("10 秒内没有任何事件", flush=True)
