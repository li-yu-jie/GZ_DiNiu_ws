# =============================================================================
# sysinfo.py — 工控机系统状态（CPU/内存/磁盘/温度/负载）
# =============================================================================
import os
import time

import psutil

_boot_time = psutil.boot_time()


def get_sysinfo():
    vm = psutil.virtual_memory()
    info = {
        'cpu_percent': psutil.cpu_percent(interval=None),
        'mem_percent': vm.percent,
        'mem_used_gb': round(vm.used / 1e9, 1),
        'mem_total_gb': round(vm.total / 1e9, 1),
        'disk_percent': psutil.disk_usage('/').percent,
        'load_avg': [round(x, 2) for x in os.getloadavg()],
        'uptime_hours': round((time.time() - _boot_time) / 3600, 1),
        'temperatures': {},
    }
    try:
        temps = psutil.sensors_temperatures()
        for name, entries in temps.items():
            if entries:
                info['temperatures'][name] = round(entries[0].current, 1)
    except Exception:
        pass
    return info
